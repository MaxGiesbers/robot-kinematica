#include "Matrix.hpp"

namespace Kinematics
{
template<typename T, std::size_t m, std::size_t n>
Matrix<T, m, n>::Matrix(T value)
{
	for (std::size_t row = 0; row < m; ++row)
	{
		for (std::size_t column = 0; column < n; ++column)
		{
			matrix.at(row).at(column) = value;
		}
	}
}

template<typename T, std::size_t m, std::size_t n>
Matrix<T, m, n>::Matrix(const std::initializer_list<std::initializer_list<T>>& list)
{
	for (std::size_t row = 0; row < m; ++row)
	{
		for (std::size_t column = 0; column < n; ++column)
		{
			matrix[row][column] = list.begin()[row].begin()[column];
		}
	}
}

template<typename T, std::size_t m, std::size_t n>
std::array<T, n>& Matrix<T, m, n>::operator[](std::size_t index)
{
	return matrix[index];
}

template<typename T, std::size_t m, std::size_t n>
const std::array<T, n>& Matrix<T, m, n>::operator[](std::size_t index) const
{
	return matrix[index];
}

template<typename T, std::size_t m, std::size_t n>
std::array<T, n>& Matrix<T, m, n>::at(std::size_t row)
{
	return matrix.at(row);
}

template<typename T, std::size_t m, std::size_t n>
const std::array<T, n>& Matrix<T, m, n>::at(std::size_t row) const
{
	return matrix.at(row);
}

template<typename T, std::size_t m, std::size_t n>
T& Matrix<T, m, n>::at(std::size_t row, std::size_t column)
{
	return matrix.at(row).at(column);
}

template<typename T, std::size_t m, std::size_t n>
const T& Matrix<T, m, n>::at(std::size_t row, std::size_t column) const
{
	return matrix.at(row).at(column);
}

template<typename T, std::size_t m, std::size_t n>
std::size_t Matrix<T, m, n>::get_m() const
{
	return m;
}

template<typename T, std::size_t m, std::size_t n>
std::size_t Matrix<T, m, n>::get_n() const
{
	return n;
}

template<typename T, std::size_t m, std::size_t n>
std::string Matrix<T, m, n>::to_string() const
{
	std::string output = "";

	for (std::size_t row = 0; row < m; ++row)
	{
		for (std::size_t column = 0; column < n; ++column)
		{
			output += (std::to_string(matrix.at(row).at(column)) + "\t");
		}
		output += "\n";
	}
	return output.substr(0, output.size() - 1);
}

template<typename T, std::size_t m, std::size_t n>
bool Matrix<T, m, n>::floating_point_equal(const Matrix<T, m, n>& mat, T floating_point) const
{
	for (std::size_t row = 0; row < m; ++row)
	{
		for (std::size_t column = 0; column < n; ++column)
		{
			if ((std::abs(matrix.at(row).at(column)) - std::abs(mat.at(row).at(column)))
			    > floating_point)
			{
				return false;
			}
		}
	}
	return true;
}

template<typename T, std::size_t m, std::size_t n>
bool Matrix<T, m, n>::operator==(const Matrix<T, m, n>& mat) const
{
	return floating_point_equal(mat, 0);
}

template<typename T, std::size_t m, std::size_t n>
bool Matrix<T, m, n>::operator!=(const Matrix<T, m, n>& mat) const
{
	return !floating_point_equal(mat, 0);
}

template<typename T, std::size_t m, std::size_t n>
void Matrix<T, m, n>::operator+=(const T& scalar)
{
	for (std::size_t row = 0; row < m; ++row)
	{
		for (std::size_t column = 0; column < n; ++column)
		{
			matrix.at(row).at(column) += scalar;
		}
	}
}

template<typename T, std::size_t m, std::size_t n>
void Matrix<T, m, n>::operator-=(const T& scalar)
{
	for (std::size_t row = 0; row < m; ++row)
	{
		for (std::size_t column = 0; column < n; ++column)
		{
			matrix.at(row).at(column) -= scalar;
		}
	}
}

template<typename T, std::size_t m, std::size_t n>
void Matrix<T, m, n>::operator*=(const T& scalar)
{
	for (std::size_t row = 0; row < m; ++row)
	{
		for (std::size_t column = 0; column < n; ++column)
		{
			matrix.at(row).at(column) *= scalar;
		}
	}
}

template<typename T, std::size_t m, std::size_t n>
void Matrix<T, m, n>::operator/=(const T& scalar)
{
	for (std::size_t row = 0; row < m; ++row)
	{
		for (std::size_t column = 0; column < n; ++column)
		{
			matrix.at(row).at(column) /= scalar;
		}
	}
}

template<typename T, std::size_t m, std::size_t n>
void Matrix<T, m, n>::operator+=(const Matrix<T, m, n>& mat)
{
	for (std::size_t row = 0; row < m; ++row)
	{
		for (std::size_t column = 0; column < n; ++column)
		{
			matrix.at(row).at(column) += mat.at(row).at(column);
		}
	}
}

template<typename T, std::size_t m, std::size_t n>
void Matrix<T, m, n>::operator-=(const Matrix<T, m, n>& mat)
{
	for (std::size_t row = 0; row < m; ++row)
	{
		for (std::size_t column = 0; column < n; ++column)
		{
			matrix.at(row).at(column) -= mat.at(row).at(column);
		}
	}
}

template<typename T, std::size_t m, std::size_t n>
Matrix<T, m, n> Matrix<T, m, n>::operator+(const T& scalar) const
{
	Matrix<T, m, n> output(*this);
	for (std::size_t row = 0; row < m; ++row)
	{
		for (std::size_t column = 0; column < n; ++column)
		{
			output.at(row).at(column) += scalar;
		}
	}
	return output;
}

template<typename T, std::size_t m, std::size_t n>
Matrix<T, m, n> Matrix<T, m, n>::operator-(const T& scalar) const
{
	Matrix<T, m, n> output(*this);
	for (std::size_t row = 0; row < m; ++row)
	{
		for (std::size_t column = 0; column < n; ++column)
		{
			output.at(row).at(column) -= scalar;
		}
	}
	return output;
}

template<typename T, std::size_t m, std::size_t n>
Matrix<T, m, n> Matrix<T, m, n>::operator*(const T& scalar) const
{
	Matrix<T, m, n> output(*this);
	for (std::size_t row = 0; row < m; ++row)
	{
		for (std::size_t column = 0; column < n; ++column)
		{
			output.at(row).at(column) *= scalar;
		}
	}
	return output;
}

template<typename T, std::size_t m, std::size_t n>
Matrix<T, m, n> Matrix<T, m, n>::operator/(const T& scalar) const
{
	Matrix<T, m, n> output(*this);
	for (std::size_t row = 0; row < m; ++row)
	{
		for (std::size_t column = 0; column < n; ++column)
		{
			output.at(row).at(column) /= scalar;
		}
	}
	return output;
}

template<typename T, std::size_t m, std::size_t n>
Matrix<T, m, n> Matrix<T, m, n>::operator+(const Matrix<T, m, n>& mat) const
{
	Matrix<T, m, n> output(*this);
	for (std::size_t row = 0; row < m; ++row)
	{
		for (std::size_t column = 0; column < n; ++column)
		{
			output.at(row).at(column) += mat.at(row).at(column);
		}
	}
	return output;
}

template<typename T, std::size_t m, std::size_t n>
Matrix<T, m, n> Matrix<T, m, n>::operator-(const Matrix<T, m, n>& mat) const
{
	Matrix<T, m, n> output(*this);
	for (std::size_t row = 0; row < m; ++row)
	{
		for (std::size_t column = 0; column < n; ++column)
		{
			output.at(row).at(column) -= mat.at(row).at(column);
		}
	}
	return output;
}

template<typename T, std::size_t m, std::size_t n>
template<std::size_t r>
Matrix<T, m, r> Matrix<T, m, n>::operator*(const Matrix<T, n, r>& mat) const
{
	Matrix<T, m, r> output;

	for (std::size_t row = 0; row < m; ++row)
	{
		for (std::size_t pos = 0; pos < r; ++pos)
		{
			for (std::size_t column = 0; column < n; ++column)
			{
				output.at(row).at(pos) += matrix.at(row).at(column) * mat.at(column).at(pos);
			}
		}
	}

	return output;
}

template<typename T, std::size_t m, std::size_t n>
T Matrix<T, m, n>::get_magnitude() const
{
	static_assert(m == 1 || n == 1, "Magnitude is only defined for column and row matrices.");

	T magnitude = 0;

	if (m == 1)
	{
		magnitude = static_cast<T>(std::sqrt(((*this) * this->transpose())[0][0]));
	}
	else
	{
		magnitude = static_cast<T>(std::sqrt((this->transpose() * (*this))[0][0]));
	}
	return magnitude;
}

template<typename T, std::size_t m, std::size_t n>
Matrix<T, m, n> Matrix<T, m, n>::get_identity() const
{
	Matrix<T, m, n> output(*this);
	for (std::size_t row = 0; row < m; ++row)
	{
		for (std::size_t column = 0; column < n; ++column)
		{
			if (row == column)
			{
				output.at(row).at(column) = 1;
			}
			else
			{
				output.at(row).at(column) = 0;
			}
		}
	}
	return output;
}

template<typename T, std::size_t m, std::size_t n>
Matrix<T, n, m> Matrix<T, m, n>::transpose() const
{
	Matrix<T, n, m> output;
	for (std::size_t row = 0; row < m; ++row)
	{
		for (std::size_t column = 0; column < n; ++column)
		{
			output.at(column).at(row) = matrix.at(row).at(column);
		}
	}
	return output;
}

template<typename T, std::size_t m, std::size_t n>
std::optional<Matrix<T, m, n>> Matrix<T, m, n>::inverse() const
{
	static_assert(m == n, "Inverse can only be computed for square matrices");

	auto augmented = augment(get_identity());

	for (std::size_t i = 0; i < m; ++i)
	{
		std::size_t min_row_index = i;
		std::optional<T> closest_to_one;

		for (std::size_t j = min_row_index; j < m; ++j)
		{
			if (std::abs(augmented[j][i]) <= std::numeric_limits<T>::min())
			{
				continue;
			}

			const T abs_value = std::abs(1 - std::abs(augmented[j][i]));

			if (!closest_to_one.has_value() || abs_value < closest_to_one.value())
			{
				closest_to_one = abs_value;
				min_row_index = j;
			}
		}

		if (!closest_to_one.has_value())
		{
			return std::nullopt;
		}

		if (min_row_index != i)
		{
			for (std::size_t j = 0; j < 2 * m; ++j)
			{
				T tmp = augmented[i][j];
				augmented[i][j] = augmented[min_row_index][j];
				augmented[min_row_index][j] = tmp;
			}
		}

		const auto divisor = augmented[i][i];

		for (std::size_t j = 0; j < 2 * m; ++j)
		{
			augmented[i][j] /= divisor;
		}

		for (std::size_t j = 0; j < m; ++j)
		{
			if (i == j)
			{
				continue;
			}

			const auto multiplier = augmented[j][i];

			for (std::size_t k = 0; k < 2 * m; ++k)
			{
				augmented[j][k] -= multiplier * augmented[i][k];
			}
		}
	}

	Matrix<T, m, n> inverse;
	for (std::size_t row = 0; row < m; ++row)
	{
		for (std::size_t column = 0; column < n; ++column)
		{
			inverse[row][column] = augmented[row][column + n];
		}
	}

	const auto identity_check = inverse * (*this);
	const auto equal_precision = 0.0000000001;
	const auto identity = inverse.get_identity();

	if (!identity_check.floating_point_equal(identity, equal_precision))
	{
		return std::nullopt;
	}

	return inverse;
}

template<typename T, std::size_t m, std::size_t n>
template<std::size_t r>
Matrix<T, m, n + r> Matrix<T, m, n>::augment(const Matrix<T, r, n>& mat) const
{
	Matrix<T, m, n + r> augmented;
	for (std::size_t row = 0; row < m; ++row)
	{
		for (std::size_t column = 0; column < n; ++column)
		{
			augmented.at(row).at(column) = matrix.at(row).at(column);
		}

		for (std::size_t column = n; column < n + r; ++column)
		{
			augmented.at(row).at(column) = mat.at(row).at(column - n);
		}
	}
	return augmented;
}

template<typename T, std::size_t m, std::size_t n>
template<std::size_t y, std::size_t x, std::size_t p, std::size_t q>
Matrix<T, p, q> Matrix<T, m, n>::slice() const
{
	Matrix<T, p, q> mat;

	for (std::size_t i = y; i < m && i - y < p; ++i)
	{
		for (std::size_t j = x; j < n && j - x < q; ++j)
		{
			mat[i - y][j - x] = (*this)[i][j];
		}
	}
	return mat;
}

template<typename T, std::size_t m, std::size_t n>
std::ostream& operator<<(std::ostream& os, const Matrix<T, m, n>& mat)
{
	return os << mat.to_string();
}
template<typename T, const std::size_t m, const std::size_t n>
Matrix<T, m, n> operator+(const Matrix<T, m, n>& mat, const T scalar)
{
	Matrix<T, m, n> output(mat);
	output += scalar;
	return output;
}

template<typename T, const std::size_t m, const std::size_t n>
Matrix<T, m, n> operator+(const T scalar, const Matrix<T, m, n>& mat)
{
	Matrix<T, m, n> output(mat);
	output += scalar;
	return output;
}

template<typename T, const std::size_t m, const std::size_t n>
Matrix<T, m, n> operator-(const Matrix<T, m, n>& mat, const T scalar)
{
	Matrix<T, m, n> output(mat);
	output -= scalar;
	return output;
}

template<typename T, const std::size_t m, const std::size_t n>
Matrix<T, m, n> operator-(const T scalar, const Matrix<T, m, n>& mat)
{
	Matrix<T, m, n> output(mat);
	output -= scalar;
	return output;
}

template<typename T, const std::size_t m, const std::size_t n>
Matrix<T, m, n> operator*(const Matrix<T, m, n>& mat, const T scalar)
{
	Matrix<T, m, n> output(mat);
	output *= scalar;
	return output;
}

template<typename T, const std::size_t m, const std::size_t n>
Matrix<T, m, n> operator*(const T scalar, const Matrix<T, m, n>& mat)
{
	Matrix<T, m, n> output(mat);
	output *= scalar;
	return output;
}

template<typename T, const std::size_t m, const std::size_t n>
Matrix<T, m, n> operator/(const Matrix<T, m, n>& mat, const T scalar)
{
	Matrix<T, m, n> output(mat);
	output /= scalar;
	return output;
}

template<typename T, const std::size_t m, const std::size_t n>
Matrix<T, m, n> operator/(const T scalar, const Matrix<T, m, n>& mat)
{
	Matrix<T, m, n> output(mat);
	output /= scalar;
	return output;
}
} // namespace Kinematics