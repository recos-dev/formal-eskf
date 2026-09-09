#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string_view>

#include <formal_eskf/linalg/backend/eigen.hpp>

namespace
{

class TestContext
{
public:
    void expect(bool condition, std::string_view profile, std::string_view description)
    {
        if (!condition)
        {
            ++m_failures;
            std::cerr << "FAIL [" << profile << "]: " << description << '\n';
        }
    }

    [[nodiscard]] int failures() const noexcept { return m_failures; }

private:
    int m_failures = 0;

}; /* end class TestContext */

template <typename T> [[nodiscard]] bool near(T actual, T expected, T tolerance)
{
    using std::abs;
    return abs(actual - expected) <= tolerance;
}

template <typename Matrix>
[[nodiscard]] bool matrix_near(Matrix const & actual, Matrix const & expected, typename Matrix::value_type tolerance)
{
    for (std::size_t row = 0U; row < Matrix::row_count; ++row)
    {
        for (std::size_t column = 0U; column < Matrix::column_count; ++column)
        {
            if (!near(actual(row, column), expected(row, column), tolerance))
            {
                return false;
            }
        }
    }
    return true;
}

template <typename Linalg>
void test_construction_and_access(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using value_type = typename Linalg::value_type;
    using matrix23_type = typename Linalg::template matrix_type<2U, 3U>;
    using matrix22_type = typename Linalg::template matrix_type<2U, 2U>;
    using matrix34_type = typename Linalg::template matrix_type<3U, 4U>;
    using vector4_type = typename Linalg::template vector_type<4U>;
    using vector2_type = typename Linalg::template vector_type<2U>;

    constexpr value_type coefficients[6] = {1, 2, 3, 4, 5, 6};
    matrix23_type matrix = matrix23_type::from_row_major(coefficients);
    test.expect(near(matrix(0U, 0U), value_type{1}, tolerance), profile, "row-major construction starts at (0,0)");
    test.expect(near(matrix(1U, 2U), value_type{6}, tolerance), profile, "row-major construction ends at (1,2)");

    matrix(0U, 1U) = value_type{8};
    test.expect(near(matrix(0U, 1U), value_type{8}, tolerance), profile, "coefficient access is mutable");

    matrix22_type const identity = matrix22_type::identity();
    test.expect(near(identity(0U, 0U), value_type{1}, tolerance) && near(identity(0U, 1U), value_type{0}, tolerance) &&
                    near(identity(1U, 1U), value_type{1}, tolerance),
                profile, "identity has ones only on the diagonal");

    constexpr value_type block_values[4] = {9, 8, 7, 6};
    matrix22_type const block = matrix22_type::from_row_major(block_values);
    matrix34_type block_target;
    block_target.template set_block<1U, 1U>(block);
    matrix22_type const block_copy = block_target.template block<1U, 1U, 2U, 2U>();
    test.expect(matrix_near(block_copy, block, tolerance), profile, "compile-time block set/get round trip");

    constexpr value_type vector_values[4] = {1, 2, 3, 4};
    vector4_type vector = vector4_type::from_row_major(vector_values);
    vector2_type const segment = vector.template segment<1U, 2U>();
    test.expect(near(segment(0U), value_type{2}, tolerance) && near(segment(1U), value_type{3}, tolerance), profile,
                "compile-time segment extraction");

    constexpr value_type replacement_values[2] = {7, 8};
    vector2_type const replacement = vector2_type::from_row_major(replacement_values);
    vector.template set_segment<1U>(replacement);
    test.expect(near(vector(0U), value_type{1}, tolerance) && near(vector(1U), value_type{7}, tolerance) &&
                    near(vector(2U), value_type{8}, tolerance) && near(vector(3U), value_type{4}, tolerance),
                profile, "compile-time segment replacement changes only its target");
}

template <typename Linalg>
void test_arithmetic_and_products(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using value_type = typename Linalg::value_type;
    using matrix23_type = typename Linalg::template matrix_type<2U, 3U>;
    using matrix32_type = typename Linalg::template matrix_type<3U, 2U>;
    using matrix22_type = typename Linalg::template matrix_type<2U, 2U>;
    using vector3_type = typename Linalg::template vector_type<3U>;

    constexpr value_type left_values[6] = {1, 2, 3, 4, 5, 6};
    constexpr value_type right_values[6] = {7, 8, 9, 10, 11, 12};
    matrix23_type const left = matrix23_type::from_row_major(left_values);
    matrix32_type const right = matrix32_type::from_row_major(right_values);
    matrix22_type const product = left * right;

    constexpr value_type expected_values[4] = {58, 64, 139, 154};
    matrix22_type const expected = matrix22_type::from_row_major(expected_values);
    test.expect(matrix_near(product, expected, tolerance), profile, "matrix multiplication");

    auto const transposed = transpose(left);
    test.expect(near(transposed(2U, 1U), value_type{6}, tolerance), profile, "matrix transpose");

    matrix23_type const recovered = (left + left) / value_type{2};
    test.expect(matrix_near(recovered, left, tolerance), profile, "eager addition and scalar division");

    matrix23_type const zero = left + (-left);
    test.expect(near(max_abs(zero), value_type{0}, tolerance), profile, "eager unary negation");

    matrix23_type const scaled_left = value_type{3} * left;
    matrix23_type const scaled_right = left * value_type{3};
    test.expect(matrix_near(scaled_left, scaled_right, tolerance), profile, "scalar multiplication on both sides");

    constexpr value_type vector_values[3] = {1, 2, 2};
    vector3_type const vector = vector3_type::from_row_major(vector_values);
    test.expect(near(dot(vector, vector), value_type{9}, tolerance), profile, "dot product");
    test.expect(near(squared_norm(vector), value_type{9}, tolerance), profile, "squared norm");
    test.expect(near(norm(vector), value_type{3}, tolerance), profile, "norm");

    constexpr value_type x_values[3] = {1, 0, 0};
    constexpr value_type y_values[3] = {0, 1, 0};
    vector3_type const x = vector3_type::from_row_major(x_values);
    vector3_type const y = vector3_type::from_row_major(y_values);
    vector3_type const z = cross(x, y);
    test.expect(near(z(0U), value_type{0}, tolerance) && near(z(1U), value_type{0}, tolerance) &&
                    near(z(2U), value_type{1}, tolerance),
                profile, "three-dimensional cross product");
}

template <typename Linalg>
void test_derived_operations(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using value_type = typename Linalg::value_type;
    using matrix22_type = typename Linalg::template matrix_type<2U, 2U>;
    using matrix12_type = typename Linalg::template matrix_type<1U, 2U>;
    using vector2_type = typename Linalg::template vector_type<2U>;

    constexpr value_type matrix_values[4] = {1, 4, 2, 3};
    matrix22_type const matrix = matrix22_type::from_row_major(matrix_values);
    test.expect(near(trace(matrix), value_type{4}, tolerance), profile, "trace");

    vector2_type const matrix_diagonal = diagonal(matrix);
    test.expect(near(matrix_diagonal(0U), value_type{1}, tolerance) &&
                    near(matrix_diagonal(1U), value_type{3}, tolerance),
                profile, "diagonal");

    matrix22_type const symmetric = symmetrize(matrix);
    test.expect(is_symmetric(symmetric, value_type{0}), profile, "exact symmetric matrix detection");
    test.expect(!is_symmetric(matrix, value_type{0.5}), profile, "asymmetric matrix detection");

    constexpr value_type transform_values[2] = {2, 3};
    constexpr value_type covariance_values[4] = {4, 1, 1, 2};
    matrix12_type const transform = matrix12_type::from_row_major(transform_values);
    matrix22_type const covariance = matrix22_type::from_row_major(covariance_values);
    auto const projected = sandwich(transform, covariance);
    test.expect(near(projected(0U, 0U), value_type{46}, tolerance), profile, "sandwich product A*P*A^T");

    constexpr value_type vector_values[2] = {3, 4};
    vector2_type const input = vector2_type::from_row_major(vector_values);
    vector2_type normalized;
    formal_eskf::Status const status = try_normalize(input, tolerance, normalized);
    test.expect(status == formal_eskf::Status::success &&
                    near(normalized(0U), value_type{3} / value_type{5}, tolerance) &&
                    near(normalized(1U), value_type{4} / value_type{5}, tolerance),
                profile, "safe vector normalization");
}

template <typename Linalg> void test_finite_checks(TestContext & test, std::string_view profile)
{
    using value_type = typename Linalg::value_type;
    using matrix22_type = typename Linalg::template matrix_type<2U, 2U>;

    matrix22_type matrix = matrix22_type::identity();
    test.expect(all_finite(matrix), profile, "finite matrix inspection");

    matrix(0U, 1U) = std::numeric_limits<value_type>::quiet_NaN();
    test.expect(!all_finite(matrix), profile, "NaN matrix inspection");
    test.expect(std::isnan(max_abs(matrix)), profile, "max_abs propagates NaN");
}

template <typename Linalg>
void test_spd_solves(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using value_type = typename Linalg::value_type;
    using matrix22_type = typename Linalg::template matrix_type<2U, 2U>;
    using matrix32_type = typename Linalg::template matrix_type<3U, 2U>;

    constexpr value_type system_values[4] = {4, 1, 1, 3};
    constexpr value_type rhs_values[4] = {1, 2, 3, 4};
    matrix22_type const system = matrix22_type::from_row_major(system_values);
    matrix22_type const rhs = matrix22_type::from_row_major(rhs_values);
    matrix22_type solution;

    formal_eskf::Status const status = solve_spd(system, rhs, solution);
    matrix22_type const residual = system * solution - rhs;
    test.expect(status == formal_eskf::Status::success && max_abs(residual) < tolerance, profile,
                "left SPD solve satisfies S*X=B");

    constexpr value_type right_rhs_values[6] = {1, 2, 3, 4, 5, 6};
    matrix32_type const right_rhs = matrix32_type::from_row_major(right_rhs_values);
    matrix32_type right_solution;
    formal_eskf::Status const right_status = right_solve_spd(right_rhs, system, right_solution);
    matrix32_type const right_residual = right_solution * system - right_rhs;
    test.expect(right_status == formal_eskf::Status::success && max_abs(right_residual) < tolerance, profile,
                "right SPD solve satisfies X*S=B");

    constexpr value_type non_spd_values[4] = {1, 0, 0, -1};
    matrix22_type const non_spd = matrix22_type::from_row_major(non_spd_values);
    matrix22_type unchanged = matrix22_type::identity();
    matrix22_type const expected_unchanged = unchanged;
    formal_eskf::Status const non_spd_status = solve_spd(non_spd, rhs, unchanged);
    test.expect(non_spd_status == formal_eskf::Status::not_positive_definite, profile, "non-SPD system fails");
    test.expect(matrix_near(unchanged, expected_unchanged, tolerance), profile, "failed solve preserves output");

    constexpr value_type asymmetric_values[4] = {4, 100, 1, 3};
    matrix22_type const asymmetric = matrix22_type::from_row_major(asymmetric_values);
    formal_eskf::Status const asymmetric_status = solve_spd(asymmetric, rhs, unchanged);
    test.expect(asymmetric_status == formal_eskf::Status::not_positive_definite, profile,
                "asymmetric system cannot report SPD success");
    test.expect(matrix_near(unchanged, expected_unchanged, tolerance), profile, "asymmetric solve preserves output");
}

template <typename Linalg>
void test_eskf_dimensions(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    using value_type = typename Linalg::value_type;
    using matrix15_type = typename Linalg::template matrix_type<15U, 15U>;
    using vector15_type = typename Linalg::template vector_type<15U>;

    matrix15_type const system = matrix15_type::identity();
    vector15_type right_hand_side;
    for (std::size_t index = 0U; index < 15U; ++index)
    {
        right_hand_side(index) = static_cast<value_type>(index + 1U);
    }

    vector15_type solution;
    formal_eskf::Status const status = solve_spd(system, right_hand_side, solution);
    test.expect(status == formal_eskf::Status::success && matrix_near(solution, right_hand_side, tolerance), profile,
                "15x15 SPD solve uses fixed storage without allocation");
}

template <typename Linalg>
void run_conformance(TestContext & test, std::string_view profile, typename Linalg::value_type tolerance)
{
    test_construction_and_access<Linalg>(test, profile, tolerance);
    test_arithmetic_and_products<Linalg>(test, profile, tolerance);
    test_derived_operations<Linalg>(test, profile, tolerance);
    test_finite_checks<Linalg>(test, profile);
    test_spd_solves<Linalg>(test, profile, tolerance);
    test_eskf_dimensions<Linalg>(test, profile, tolerance);
}

} /* end namespace */

int main()
{
    TestContext test;

    Eigen::internal::set_is_malloc_allowed(false);
    run_conformance<formal_eskf::linalg::EigenBackend<double>>(test, "binary64", 1.0e-12);
    run_conformance<formal_eskf::linalg::EigenBackend<float>>(test, "binary32", 1.0e-5F);

    if (test.failures() != 0)
    {
        std::cerr << test.failures() << " linalg test(s) failed\n";
        return 1;
    }

    std::cout << "All Eigen linalg backend conformance tests passed\n";
    return 0;
}
