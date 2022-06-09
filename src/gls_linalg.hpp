// Copyright (c) 2021-2022 Glass Imaging Inc.
// Author: Fabio Riccardi <fabio@glass-imaging.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef gls_linalg_h
#define gls_linalg_h

#include <array>
#include <span>
#include <vector>
#include <cassert>

#include <iostream>
#include <stdexcept>

namespace gls {

template<size_t M, size_t N, typename baseT = float> struct Matrix;

// ---- Vector Type ----
template <size_t N, typename baseT = float>
struct Vector : public std::array<baseT, N> {
    Vector() { }

    Vector(const baseT(&il)[N]) {
        std::copy(il, il + N, this->begin());
    }

    Vector(const std::vector<baseT>& v) {
        assert(v.size() == N);
        std::copy(v.begin(), v.end(), this->begin());
    }

    Vector(const std::array<baseT, N>& v) {
        assert(v.size() == N);
        std::copy(v.begin(), v.end(), this->begin());
    }

    Vector(std::initializer_list<baseT> list) {
        assert(list.size() == N);
        std::copy(list.begin(), list.end(), this->begin());
    }

    template<size_t P, size_t Q>
    requires (P * Q == N)
    Vector(const Matrix<P, Q>& m) {
        const auto ms = m.span();
        std::copy(ms.begin(), ms.end(), this->begin());
    }

    template <typename T>
    Vector& operator += (const T& v) {
        *this = *this + v;
        return *this;
    }

    template <typename T>
    Vector& operator -= (const T& v) {
        *this = *this - v;
        return *this;
    }

    template <typename T>
    Vector& operator *= (const T& v) {
        *this = *this * v;
        return *this;
    }

    template <typename T>
    Vector& operator /= (const T& v) {
        *this = *this / v;
        return *this;
    }

    // Cast to a const baseT*
    operator const baseT*() const {
        return this->data();
    }
};

template <size_t N>
struct DVector : public Vector<N, double> { };

// Vector - Vector Addition (component-wise)
template <size_t N, typename baseT>
inline Vector<N, baseT> operator + (const Vector<N, baseT>& a, const Vector<N, baseT>& b) {
    auto ita = a.begin();
    auto itb = b.begin();
    Vector<N, baseT> result;
    std::for_each(result.begin(), result.end(), [&](baseT &r){ r = *ita++ + *itb++; });
    return result;
}

// Vector - Vector Subtraction (component-wise)
template <size_t N, typename baseT>
inline Vector<N, baseT> operator - (const Vector<N, baseT>& a, const Vector<N, baseT>& b) {
    auto ita = a.begin();
    auto itb = b.begin();
    Vector<N, baseT> result;
    std::for_each(result.begin(), result.end(), [&](baseT &r){ r = *ita++ - *itb++; });
    return result;
}

// Vector - Vector Multiplication (component-wise)
template <size_t N, typename baseT>
inline Vector<N, baseT> operator * (const Vector<N, baseT>& a, const Vector<N, baseT>& b) {
    auto ita = a.begin();
    auto itb = b.begin();
    Vector<N, baseT> result;
    std::for_each(result.begin(), result.end(), [&](baseT &r){ r = *ita++ * *itb++; });
    return result;
}

// Vector - Vector Division (component-wise)
template <size_t N, typename baseT>
inline Vector<N, baseT> operator / (const Vector<N, baseT>& a, const Vector<N, baseT>& b) {
    auto ita = a.begin();
    auto itb = b.begin();
    Vector<N, baseT> result;
    std::for_each(result.begin(), result.end(), [&](baseT &r){ r = *ita++ / *itb++; });
    return result;
}

// Vector - Scalar Addition
template <size_t N, typename baseT>
inline Vector<N, baseT> operator + (const Vector<N, baseT>& v, baseT a) {
    auto itv = v.begin();
    Vector<N, baseT> result;
    std::for_each(result.begin(), result.end(), [&a, &itv](baseT &r){ r = *itv++ + a; });
    return result;
}

// Vector - Scalar Addition (commutative)
template <size_t N, typename baseT>
inline Vector<N, baseT> operator + (baseT a, const Vector<N, baseT>& v) {
    return v + a;
}

// Vector - Scalar Subtraction
template <size_t N, typename baseT>
inline Vector<N, baseT> operator - (const Vector<N, baseT>& v, baseT a) {
    auto itv = v.begin();
    Vector<N, baseT> result;
    std::for_each(result.begin(), result.end(), [&a, &itv](baseT &r){ r = *itv++ - a; });
    return result;
}

// Scalar - Vector Subtraction
template <size_t N, typename baseT>
inline Vector<N, baseT> operator - (baseT a, const Vector<N, baseT>& v) {
    auto itv = v.begin();
    Vector<N, baseT> result;
    std::for_each(result.begin(), result.end(), [&a, &itv](baseT &r){ r = a - *itv++; });
    return result;
}

// Vector - Scalar Multiplication
template <size_t N, typename baseT>
inline Vector<N, baseT> operator * (const Vector<N, baseT>& v, baseT a) {
    auto itv = v.begin();
    Vector<N, baseT> result;
    std::for_each(result.begin(), result.end(), [&a, &itv](baseT &r){ r = *itv++ * a; });
    return result;
}

// Scalar - Vector Multiplication (commutative)
template <size_t N, typename baseT>
inline Vector<N, baseT> operator * (baseT a, const Vector<N, baseT>& v) {
    return v * a;
}

// Vector - Scalar Division
template <size_t N, typename baseT>
inline Vector<N, baseT> operator / (const Vector<N, baseT>& v, baseT a) {
    auto itv = v.begin();
    Vector<N, baseT> result;
    std::for_each(result.begin(), result.end(), [&a, &itv](baseT &r){ r = *itv++ / a; });
    return result;
}

// Scalar - Vector Division
template <size_t N, typename baseT>
inline Vector<N, baseT> operator / (baseT a, const Vector<N, baseT>& v) {
    auto itv = v.begin();
    Vector<N, baseT> result;
    std::for_each(result.begin(), result.end(), [&a, &itv](baseT &r){ r = a / *itv++; });
    return result;
}

template <size_t N, typename baseT>
inline Vector<N, baseT> abs(const Vector<N, baseT>& v) {
    auto itv = v.begin();
    Vector<N, baseT> result;
    std::for_each(result.begin(), result.end(), [&itv](baseT &r){ r = std::abs(*itv++); });
    return result;
}

// ---- Matrix Type ----

template <size_t N, size_t M, typename baseT>
struct Matrix : public std::array<Vector<M, baseT>, N> {
    Matrix() {}

    Matrix(const Vector<N * M, baseT>& v) {
        std::copy(v.begin(), v.end(), span().begin());
    }

    Matrix(const baseT(&il)[N * M]) {
        std::copy(il, il + (N * M), span().begin());
    }

    Matrix(const std::array<baseT, M>(&il)[N]) {
        // This is safe, il is just an array of arrays
        std::copy((baseT *) il, (baseT *) il + (N * M), span().begin());
    }

    Matrix(const std::vector<baseT>& v) {
        assert(v.size() == N * M);
        std::copy(v.begin(), v.end(), span().begin());
    }

    Matrix(std::initializer_list<baseT> list) {
        assert(list.size() == N * M);
        std::copy(list.begin(), list.end(), span().begin());
    }

    Matrix(std::initializer_list<std::array<baseT, M>> list) {
        assert(list.size() == N);
        int row = 0;
        for (const auto& v : list) {
            std::copy(v.begin(), v.end(), span(row++).begin());
        }
    }

    // Matrix Raw Data
    std::span<baseT> span() {
        return std::span(&(*this)[0][0], N * M);
    }

    const std::span<const baseT> span() const {
        return std::span(&(*this)[0][0], N * M);
    }

    // Matrix Row Raw Data
    std::span<baseT> span(int row) {
        return std::span(&(*this)[row][0], M);
    }

    const std::span<const baseT> span(int row) const {
        return std::span(&(*this)[row][0], M);
    }

    // Cast to a const baseT*
    operator const baseT*() const {
        return span().data();
    }

    typedef baseT (*opPtr)(baseT a, baseT b);
};

template <size_t N, size_t M>
struct DMatrix : public Matrix<N, M, double> { };

template <size_t N, size_t M, typename baseT>
std::span<baseT> span(Matrix<N, M, baseT>& m) {
    return std::span(&m[0][0], N * M);
}

template <size_t N, size_t M, typename baseT>
const std::span<const baseT> span(const Matrix<N, M, baseT>& m) {
    return std::span(&m[0][0], N * M);
}

// Matrix Transpose
template<size_t N, size_t M, typename baseT>
inline Matrix<N, M, baseT> transpose(const Matrix<M, N, baseT>& m) {
    Matrix<N, M, baseT> result;
    for (int j = 0; j < M; j++) {
        for (int i = 0; i < N; i++) {
            result[i][j] = m[j][i];
        }
    }
    return result;
}

// General Matrix Multiplication
template <size_t N, size_t K, size_t M, typename baseT>
inline Matrix<M, N, baseT> operator * (const Matrix<M, K, baseT>& a, const Matrix<K, N, baseT>& b) {
    Matrix<M, N, baseT> result;
    const auto bt = transpose(b);
    for (int j = 0; j < M; j++) {
        for (int i = 0; i < N; i++) {
            result[j][i] = 0;
            for (int k = 0; k < K; k++) {
                result[j][i] += a[j][k] * bt[i][k];
            }
        }
    }
    return result;
}

// Matrix - Vector Multiplication
template <size_t M, size_t N, typename baseT>
inline Vector<M, baseT> operator * (const Matrix<M, N, baseT>& a, const Vector<N, baseT>& b) {
    const auto result = a * Matrix<N, 1, baseT> { b };
    return Vector<M, baseT>(result);
}

// Vector - Matrix Multiplication
template <size_t M, size_t N, typename baseT>
inline Vector<N, baseT> operator * (const Vector<M, baseT>& a, const Matrix<M, N, baseT>& b) {
    const auto result = Matrix<1, N, baseT> { a } * b;
    return Vector<N, baseT>(result);
}

// (Square) Matrix Division (Multiplication with Inverse)
template <size_t N, typename baseT>
inline Matrix<N, N, baseT> operator / (const Matrix<N, N, baseT>& a, const Matrix<N, N, baseT>& b) {
    return a * inverse(b);
}

// Iterate over the elements of the input and output matrices applying a Matrix-Matrix function
template<size_t N, size_t M, typename baseT>
inline Matrix<N, M, baseT> apply(const Matrix<M, N, baseT>& a, const Matrix<M, N, baseT>& b, typename Matrix<N, M, baseT>::opPtr f) {
    Matrix<N, M, baseT> result;
    auto ita = span(a).begin();
    auto itb = span(b).begin();
    for (auto& r : span(result)) {
        r = f(*ita++, *itb++);
    }
    return result;
}

// Iterate over the elements of the input and output matrices applying a Matrix-Scalar function
template<size_t N, size_t M, typename baseT>
inline Matrix<N, M, baseT> apply(const Matrix<M, N, baseT>& a, baseT b, typename Matrix<N, M, baseT>::opPtr f) {
    Matrix<N, M, baseT> result;
    auto ita = span(a).begin();
    for (auto& r : span(result)) {
        r = f(*ita++, b);
    }
    return result;
}

// Matrix-Scalar Multiplication
template <size_t N, size_t M, typename baseT>
inline Matrix<N, M, baseT> operator * (const Matrix<N, M, baseT>& a, baseT b) {
    return apply(a, b, [](baseT a, baseT b) {
        return a * b;
    });
}

// Matrix-Scalar Division
template <size_t N, size_t M, typename baseT>
inline Matrix<N, M, baseT> operator / (const Matrix<N, M, baseT>& a, baseT b) {
    return apply(a, b, [](baseT a, baseT b) {
        return a / b;
    });
}

// Matrix-Matrix Addition
template <size_t N, size_t M, typename baseT>
inline Matrix<N, M, baseT> operator + (const Matrix<N, M, baseT>& a, const Matrix<N, M, baseT>& b) {
    return apply(a, b, [](baseT a, baseT b) {
        return a + b;
    });
}

// Matrix-Scalar Addition
template <size_t N, size_t M, typename baseT>
inline Matrix<N, M, baseT> operator + (const Matrix<N, M, baseT>& a, baseT b) {
    return apply(a, b, [](baseT a, baseT b) {
        return a + b;
    });
}

// Matrix-Matrix Subtraction
template <size_t N, size_t M, typename baseT>
inline Matrix<N, M, baseT> operator - (const Matrix<N, M, baseT>& a, const Matrix<N, M, baseT>& b) {
    return apply(a, b, [](baseT a, baseT b) {
        return a - b;
    });
}

// Matrix-Scalar Subtraction
template <size_t N, size_t M, typename baseT>
inline Matrix<N, M, baseT> operator - (const Matrix<N, M, baseT>& a, baseT b) {
    return apply(a, b, [](baseT a, baseT b) {
        return a - b;
    });
}

// --- Matrix Inverse Support ---

// Cofactor Matrix
// https://en.wikipedia.org/wiki/Minor_(linear_algebra)#Inverse_of_a_matrix
template <size_t N1, size_t N2 = N1 - 1, typename baseT>
inline Matrix<N2, N2> cofactor(const Matrix<N1, N1, baseT>& m, int p, int q) {
    assert(p < N1 && q < N1);

    Matrix<N2, N2> result;

    // Looping for each element of the matrix
    int i = 0, j = 0;
    for (int row = 0; row < N1; row++) {
        for (int col = 0; col < N1; col++) {
            //  Copying into temporary matrix only those element
            //  which are not in given row and column
            if (row != p && col != q) {
                result[i][j++] = m[row][col];

                // Row is filled, so increase row index and
                // reset col index
                if (j == N1 - 1) {
                    j = 0;
                    i++;
                }
            }
        }
    }
    return result;
}

// Matrix Determinant using Laplace's Cofactor Expansion
// https://en.wikipedia.org/wiki/Minor_(linear_algebra)#Cofactor_expansion_of_the_determinant
template <size_t N, typename baseT>
inline baseT determinant(const Matrix<N, N, baseT>& m) {
    assert(N > 1);

    baseT sign = 1;
    baseT result = 0;
    // Iterate for each element of first row
    for (int f = 0; f < N; f++) {
        result += sign * m[0][f] * determinant(cofactor(m, 0, f));
        // terms are to be added with alternate sign
        sign = -sign;
    }
    return result;
}

// Matrix Determinant, Special case for size 1x1
template <typename baseT>
inline baseT determinant(const Matrix<1, 1, baseT>& m) {
    return m[0][0];
}

// Matrix Adjoint (Tanspose of the Cofactor Matrix)
// https://en.wikipedia.org/wiki/Adjugate_matrix
template <size_t N, typename baseT>
inline Matrix<N, N, baseT> adjoint(const Matrix<N, N, baseT>& m) {
    assert(N > 1);

    Matrix<N, N, baseT> adj;

    baseT sign = 1;
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            // sign of adj[j][i] positive if sum of row
            // and column indexes is even.
            sign = ((i + j) % 2 == 0) ? 1 : -1;

            // Interchanging rows and columns to get the
            // transpose of the cofactor matrix
            baseT d = determinant(cofactor(m, i, j));
            adj[j][i] = d != 0 ? sign * d : 0;
        }
    }
    return adj;
}

// Matrix Adjoint - Special case for size 1x1
template <typename baseT>
inline Matrix<1, 1, baseT> adjoint(const Matrix<1, 1, baseT>& m) {
    return { 1 };
}

// Inverse Matrix: inverse(m) = adj(m)/det(m)
// https://en.wikipedia.org/wiki/Minor_(linear_algebra)#Inverse_of_a_matrix
template <size_t N, typename baseT>
inline Matrix<N, N, baseT> inverse(const Matrix<N, N, baseT>& m) {
    baseT det = determinant(m);
    if (det == 0) {
        throw std::range_error("null determinant");
    }

    Matrix<N, N, baseT> inverse;
    const auto adj = adjoint(m);
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) inverse[i][j] = adj[i][j] / det;

    return inverse;
}

// From DCRaw (https://www.dechifro.org/dcraw/)
template <size_t size, typename baseT>
gls::Matrix<size, 3> pseudoinverse(const gls::Matrix<size, 3, baseT>& in) {
    gls::Matrix<3,6> work;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 6; j++) {
            work[i][j] = j == i + 3;
        }
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < size; k++) work[i][j] += in[k][i] * in[k][j];
        }
    }
    for (int i = 0; i < 3; i++) {
        baseT num = work[i][i];
        for (int j = 0; j < 6; j++) work[i][j] /= num;
        for (int k = 0; k < 3; k++) {
            if (k == i) continue;
            num = work[k][i];
            for (int j = 0; j < 6; j++) work[k][j] -= work[i][j] * num;
        }
    }
    gls::Matrix<size, 3> out;
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < 3; j++) {
            out[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                out[i][j] += work[j][k + 3] * in[i][k];
            }
        }
    }
    return out;
}

// --- Utility Functions ---

template <size_t N, typename baseT>
inline std::ostream& operator<<(std::ostream& os, const Vector<N, baseT>& v) {
    for (int i = 0; i < N; i++) {
        os << v[i];
        if (i < N - 1) {
            os << ", ";
        }
    }
    return os;
}

template <size_t N, size_t M, typename baseT>
inline std::ostream& operator<<(std::ostream& os, const Matrix<N, M, baseT>& m) {
    for (int j = 0; j < N; j++) {
        os << m[j] << ",";
        if (j < N-1) {
            os << std::endl;
        }
    }
    return os;
}

}  // namespace gls

namespace std {

// Useful for printing a gls::Matrix on a single line

template <typename baseT>
inline std::ostream& operator<<(std::ostream& os, const std::span<baseT>& s) {
    for (int i = 0; i < s.size(); i++) {
        os << s[i];
        if (i < s.size() - 1) {
            os << ", ";
        }
    }
    return os;
}

} // namespace std

#endif /* gls_linalg_h */
