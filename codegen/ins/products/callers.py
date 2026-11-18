#!/usr/bin/env python3
"""Patch isolated core copies at arithmetic sites, retaining all checked callers.

The mutable test selector is deliberately not a proposed production interface.
It selects a known observation contract, never guesses the model from dimension.
"""

from pathlib import Path
import shutil

from ins.formatting import format_cpp
from ins.verify import require
from ins.verify.product_reference import LAYOUTS

ROOT = Path(__file__).resolve().parents[3]


def fixture(STATIC=None):
    LINES = [
        '#pragma once',
        '#include <cstdlib>',
        '#include <formal_eskf/linalg/operations.hpp>',
        '#include <formal_eskf/so3/operations.hpp>',
    ]
    LINES += [
        f'#include <formal_eskf/eskf/generated/ins_{G}.hpp>'
        for G in ('reset', 'innovation', 'joseph')
    ]
    LINES += [
        'namespace codegen_fixture {',
        'namespace linalg = formal_eskf::linalg;',
        'namespace so3 = formal_eskf::so3;',
        'enum class Model { generic, ' + ', '.join(LAYOUTS) + ' };',
        (
            'inline Model selected = Model::generic;'
            if STATIC is None
            else 'inline constexpr Model selected = Model::body_vector;'
        ),
        (
            'inline bool generated = true;'
            if STATIC is None
            else 'inline constexpr bool generated = ' + ('true;' if STATIC else 'false;')
        ),
        'inline unsigned calls[4]{};',
    ]
    for KIND in ('cross', 'innovation', 'joseph'):
        DIMS = 'template <typename L, std::size_t N, std::size_t M>'
        P = 'linalg::Matrix<L,N,N> const & P'
        H = 'linalg::Matrix<L,M,N> const & H'
        V = 'linalg::Matrix<L,M,M> const & V'
        C = 'linalg::Matrix<L,N,M> const & C'
        K = 'linalg::Matrix<L,N,M> const & K'
        A = 'linalg::Matrix<L,N,N> const & A'
        PARAMS = {'cross': [P, H], 'innovation': [C, H, V], 'joseph': [P, A, H, K, V]}[KIND]
        LINES += [
            DIMS + ' auto ' + KIND + '(' + ', '.join(PARAMS) + ') {',
            'using S = typename L::value_type;',
            'if constexpr(N == 15U) { if(generated) {',
        ]
        for NAME, COLS in LAYOUTS.items():
            LINES += [
                f'if constexpr(M == {len(COLS)}U) {{ if(selected == Model::{NAME}) {{',
                'linalg::Matrix<L,M,N> expected;',
            ]
            if NAME == 'body_vector':
                LINES += [
                    'linalg::Vector<L,3U> h;',
                    'h(0U)=H(2U,7U); h(1U)=H(0U,8U); h(2U)=H(1U,6U);',
                    'expected.template set_block<0U,6U>(so3::hat<L>(h));',
                ]
            else:
                LINES += [f'expected({I}U,{COL}U)=S{{1}};' for I, COL in enumerate(COLS)]
            LINES += [
                'for(std::size_t i=0;i<M;++i) { for(std::size_t j=0;j<N;++j) {',
                'if(H(i,j)!=expected(i,j)) { std::abort(); } }}',
                f'++calls[{("cross", "innovation", "joseph").index(KIND)}];',
            ]
            ARGS = {'cross': 'P', 'innovation': 'C', 'joseph': 'P,A,K,V'}[KIND]
            if NAME == 'body_vector' and KIND != 'joseph':
                ARGS += ',h'
            if KIND == 'innovation':
                ARGS += ',V'
            LINES += [f'return formal_eskf::detail::generated::ins_{NAME}_{KIND}({ARGS});', '}}']
        LINES += ['}}']
        DENSE = {
            'cross': 'P*linalg::transpose(H)',
            'innovation': 'H*C+V',
            'joseph': 'linalg::sandwich(A,P)+linalg::sandwich(K,V)',
        }[KIND]
        LINES += [f'return {DENSE};', '}']
    LINES += [
        'template <typename L,std::size_t N> auto reset(',
        'linalg::Matrix<L,N,N> const & P,linalg::Matrix<L,N,N> const & G) {',
        'if constexpr(N==15U) { if(generated) { ++calls[3];',
        'return formal_eskf::detail::generated::ins_reset(P,G.template block<6U,6U,3U,3U>()); }}',
        'return linalg::sandwich(G,P);',
        '}',
        '}',
    ]
    return format_cpp('\n'.join(LINES))


def replace(FILE, OLD, NEW):
    TEXT = FILE.read_text()
    require(TEXT.count(OLD) == 1, 'patch site changed: ' + str(FILE))
    FILE.write_text(TEXT.replace(OLD, NEW))


def prepare(DEST):
    shutil.copytree(ROOT / 'include', DEST, dirs_exist_ok=True)
    (DEST / 'codegen_fixture.hpp').write_text(fixture())
    # This differential arithmetic fixture is separate from production dispatch tests.
    replace(DEST / 'formal_eskf/eskf/codegen.hpp', '#define ESKF_CODEGEN 1', '#define ESKF_CODEGEN 0')
    INJECTION = DEST / 'formal_eskf/eskf/injection.hpp'
    replace(INJECTION, '#pragma once', '#pragma once\n#include <codegen_fixture.hpp>')
    replace(
        INJECTION,
        'auto candidate = linalg::sandwich(G, covariance);',
        '''auto candidate = codegen_fixture::reset(covariance, G);
        if constexpr (Size == 15U) {
            if (codegen_fixture::generated) {
                return publish_covariance(candidate, output);
            }
        }''',
    )
    CORRECTION = DEST / 'formal_eskf/eskf/correction.hpp'
    replace(
        CORRECTION,
        'auto const PHt = detail::correction_cross<Model>(covariance, H);',
        'auto const PHt = codegen_fixture::cross(covariance, H);',
    )
    replace(
        CORRECTION,
        'auto S = detail::correction_innovation<Model>(PHt, H, V);',
        'auto S = codegen_fixture::innovation(PHt, H, V);',
    )
    replace(
        CORRECTION,
        'auto P_corrected = correction_joseph<Model>(covariance, A, K, V);',
        'auto P_corrected = codegen_fixture::joseph(covariance, A, H, K, V);',
    )
    replace(
        CORRECTION,
        'if constexpr (ESKF_CODEGEN && Model != CorrectionModel::arbitrary_h)',
        """if (Size == 15U && codegen_fixture::generated &&
            codegen_fixture::selected != codegen_fixture::Model::generic)""",
    )
    print('Isolated checked callers:', DEST)
