"""Emit scalar SymPy expressions without reassociation."""


def cpp_expression(EXPR):
    """Emit the existing SymPy argument order without reassociation."""
    if EXPR.is_Symbol:
        return str(EXPR)
    if EXPR.is_Integer:
        return f'value_type{{{EXPR}}}'
    if EXPR.is_Add or EXPR.is_Mul:
        OPERATOR = ' + ' if EXPR.is_Add else ' * '
        return '(' + OPERATOR.join(cpp_expression(ARG) for ARG in EXPR.args) + ')'
    if EXPR.is_Pow and EXPR.exp.is_Integer and EXPR.exp > 0:
        FACTORS = [cpp_expression(EXPR.base) for _ in range(int(EXPR.exp))]
        return '(' + ' * '.join(FACTORS) + ')'
    raise ValueError(f'unsupported expression: {EXPR}')
