#ifndef MATRIX_CLASS_MATRIX_HPP
#define MATRIX_CLASS_MATRIX_HPP

#include <array>
#include <cmath>
#include <initializer_list>
#include <optional>

namespace Matrix_class
{
template<typename T, std::size_t m, std::size_t n>
class Matrix
{
	static_assert(std::is_arithmetic<T>::value, "T must be arithmetic");
	static_assert(m > 0 && n > 0, "m and n need to be greater than 0");

public:
	explicit Matrix(T value = 0);
	Matrix(const std::initializer_list<std::initializer_list<T>>& list);
	Matrix(const Matrix<T, m, n>& mat) = default;
	virtual ~Matrix() = default;
	std::array<T, n>& operator[](std::size_t index);
	const std::array<T, n>& operator[](std::size_t index) const;
	std::array<T, n>& at(std::size_t row);
	const std::array<T, n>& at(std::size_t row) const;
	T& at(std::size_t row, std::size_t column);
	const T& at(std::size_t row, std::size_t column) const;
	std::size_t get_m() const;
	std::size_t get_n() const;
	std::string to_string() const;
	Matrix<T, m, n>& operator=(const Matrix<T, m, n>& mat) = default;
	bool floating_point_equal(const Matrix<T, m, n>& mat, T floating_point) const;
	bool operator==(const Matrix<T, m, n>& mat) const;
	bool operator!=(const Matrix<T, m, n>& mat) const;
	void operator+=(const T& scalar);
	void operator-=(const T& scalar);
	void operator*=(const T& scalar);
	void operator/=(const T& scalar);
	void operator+=(const Matrix<T, m, n>& mat);
	void operator-=(const Matrix<T, m, n>& mat);
	Matrix<T, m, n> operator+(const T& scalar) const;
	Matrix<T, m, n> operator-(const T& scalar) const;
	Matrix<T, m, n> operator*(const T& scalar) const;
	Matrix<T, m, n> operator/(const T& scalar) const;
	Matrix<T, m, n> operator+(const Matrix<T, m, n>& mat) const;
	Matrix<T, m, n> operator-(const Matrix<T, m, n>& mat) const;
	template<std::size_t r>
	Matrix<T, m, r> operator*(const Matrix<T, n, r>& mat) const;
	T get_magnitude() const;
	Matrix<T, m, n> get_identity() const;
	std::optional<Matrix<T, m, n>> inverse() const;
	Matrix<T, n, m> transpose() const;
	template<std::size_t r>
	Matrix<T, m, n + r> augment(const Matrix<T, r, n>& mat) const;
	template<std::size_t y, std::size_t x, std::size_t p, std::size_t q>
	Matrix<T, p, q> slice() const;

private:
	std::array<std::array<T, n>, m> matrix;
};

template<typename T, const std::size_t m, const std::size_t n>
std::ostream& operator<<(std::ostream& os, const Matrix<T, m, n>& mat);

template<typename T, const std::size_t m, const std::size_t n>
Matrix<T, m, n> operator+(const Matrix<T, m, n>& mat, const T scalar);

template<typename T, const std::size_t m, const std::size_t n>
Matrix<T, m, n> operator+(const T scalar, const Matrix<T, m, n>& mat);

template<typename T, const std::size_t m, const std::size_t n>
Matrix<T, m, n> operator-(const Matrix<T, m, n>& mat, const T scalar);

template<typename T, const std::size_t m, const std::size_t n>
Matrix<T, m, n> operator-(const T scalar, const Matrix<T, m, n>& mat);

template<typename T, const std::size_t m, const std::size_t n>
Matrix<T, m, n> operator*(const Matrix<T, m, n>& mat, const T scalar);

template<typename T, const std::size_t m, const std::size_t n>
Matrix<T, m, n> operator*(const T scalar, const Matrix<T, m, n>& mat);

template<typename T, const std::size_t m, const std::size_t n>
Matrix<T, m, n> operator/(const Matrix<T, m, n>& mat, const T scalar);

template<typename T, const std::size_t m, const std::size_t n>
Matrix<T, m, n> operator/(const T scalar, const Matrix<T, m, n>& mat);

} // namespace Kinematics

#include "Matrix.ipp"

#endif