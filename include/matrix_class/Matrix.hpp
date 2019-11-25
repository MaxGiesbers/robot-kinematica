#ifndef MATRIX_CLASS_MATRIX_HPP
#define MATRIX_CLASS_MATRIX_HPP

/**
 * @file Matrix.hpp
 * @author Joost Kraaijeveld
 * @brief Matrix class as provided during the WoR-course in 2018
 * @version 0.1
 * @date 2018
 * 
 * @copyright Copyright (C) 2017-2018 Joost Kraaijeveld
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program; if not, write to the Free Software
 *	Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 * 
 */

#include <array>
#include <cmath>
#include <initializer_list>
#include <optional>

namespace Kinematics
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

#endif //MATRIX_HPP