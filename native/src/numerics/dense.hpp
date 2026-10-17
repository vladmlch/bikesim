#pragma once

#include "../rider/allocation.hpp"
#include "../rider/least_squares.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>

namespace bike::numerics::dense {
using Matrix = bike::rider::Matrix;
using Vector = std::vector<double>;

inline std::size_t size(std::size_t rows, std::size_t cols) {
    if (cols != 0 && rows > std::numeric_limits<std::size_t>::max() / cols)
        throw std::invalid_argument("matrix dimensions overflow");
    return rows * cols;
}
inline Matrix zeros(std::size_t rows, std::size_t cols) {
    return {rows, cols, Vector(size(rows, cols), 0.)};
}
inline bool finite(std::span<const double> x) {
    return std::ranges::all_of(x, [](double v) { return std::isfinite(v); });
}
inline void validate(const Matrix& a, std::string_view name = "matrix") {
    if (a.values.size() != size(a.rows, a.cols) || !finite(a.values))
        throw std::invalid_argument(std::string(name) + ": finite row-major matrix required");
}
inline double dot(std::span<const double> a, std::span<const double> b) {
    if (a.size() != b.size()) throw std::invalid_argument("dot: incompatible dimensions");
    double result = 0.;
    for (std::size_t i = 0; i < a.size(); ++i) result += a[i] * b[i];
    return result;
}
inline double norm(std::span<const double> a) { return std::sqrt(dot(a, a)); }
inline Vector multiply(const Matrix& a, std::span<const double> x) {
    if (a.cols != x.size() || a.values.size() != size(a.rows, a.cols))
        throw std::invalid_argument("matrix product: incompatible dimensions");
    Vector out(a.rows, 0.);
    for (std::size_t i = 0; i < a.rows; ++i)
        for (std::size_t j = 0; j < a.cols; ++j)
            out[i] += a.values[i*a.cols+j] * x[j];
    return out;
}
inline Matrix transpose(const Matrix& a) {
    Matrix out = zeros(a.cols, a.rows);
    for (std::size_t i=0; i<a.rows; ++i)
        for (std::size_t j=0; j<a.cols; ++j) out.values[j*a.rows+i] = a.values[i*a.cols+j];
    return out;
}
inline Matrix multiply(const Matrix& a, const Matrix& b) {
    if (a.cols != b.rows) throw std::invalid_argument("matrix product: incompatible dimensions");
    Matrix out = zeros(a.rows, b.cols);
    for (std::size_t i=0; i<a.rows; ++i)
        for (std::size_t j=0; j<b.cols; ++j)
            for (std::size_t k=0; k<a.cols; ++k)
                out.values[i*b.cols+j] += a.values[i*a.cols+k]*b.values[k*b.cols+j];
    return out;
}
inline Matrix identity(std::size_t n) {
    Matrix out=zeros(n,n);
    for (std::size_t i=0; i<n; ++i) out.values[i*n+i]=1.;
    return out;
}
inline void append_row(Matrix& a, std::span<const double> row) {
    if (a.cols != row.size()) throw std::invalid_argument("matrix row: incompatible dimensions");
    a.values.insert(a.values.end(), row.begin(), row.end());
    ++a.rows;
}
inline Vector add(Vector a, std::span<const double> b, double scale=1.) {
    if (a.size()!=b.size()) throw std::invalid_argument("vector sum: incompatible dimensions");
    for(std::size_t i=0;i<a.size();++i) a[i]+=scale*b[i];
    return a;
}
inline Matrix add(Matrix a, const Matrix& b, double scale=1.) {
    if(a.rows!=b.rows || a.cols!=b.cols) throw std::invalid_argument("matrix sum: incompatible dimensions");
    a.values=add(std::move(a.values),b.values,scale);
    return a;
}

// Partial-pivot LU, used only for square systems whose source asks for
// solve(), never as a rank-hiding least-squares fallback.
inline Matrix solve(Matrix a, Matrix b) {
    validate(a); validate(b);
    const std::size_t n=a.rows, nrhs=b.cols;
    if(a.cols!=n || b.rows!=n) throw std::invalid_argument("dense solve: incompatible dimensions");
    for(std::size_t k=0;k<n;++k) {
        std::size_t pivot=k;
        for(std::size_t i=k+1;i<n;++i)
            if(std::abs(a.values[i*n+k])>std::abs(a.values[pivot*n+k])) pivot=i;
        if(a.values[pivot*n+k]==0. || !std::isfinite(a.values[pivot*n+k]))
            throw std::runtime_error("dense solve: singular matrix");
        if(pivot!=k) {
            for(std::size_t j=0;j<n;++j) std::swap(a.values[k*n+j],a.values[pivot*n+j]);
            for(std::size_t j=0;j<nrhs;++j) std::swap(b.values[k*nrhs+j],b.values[pivot*nrhs+j]);
        }
        for(std::size_t i=k+1;i<n;++i) {
            const double f=a.values[i*n+k]/a.values[k*n+k];
            for(std::size_t j=k+1;j<n;++j) a.values[i*n+j]-=f*a.values[k*n+j];
            for(std::size_t j=0;j<nrhs;++j) b.values[i*nrhs+j]-=f*b.values[k*nrhs+j];
        }
    }
    for(std::size_t k=n;k>0;--k) {
        const std::size_t i=k-1;
        for(std::size_t j=0;j<nrhs;++j) {
            for(std::size_t l=i+1;l<n;++l) b.values[i*nrhs+j]-=a.values[i*n+l]*b.values[l*nrhs+j];
            b.values[i*nrhs+j]/=a.values[i*n+i];
        }
    }
    if(!finite(b.values)) throw std::runtime_error("dense solve: nonfinite solution");
    return b;
}
inline Vector solve(const Matrix& a, std::span<const double> b) {
    return solve(a,Matrix{b.size(),1,Vector(b.begin(),b.end())}).values;
}

struct LeastSquares { Matrix solution; std::size_t rank=0; Vector singular_values; };
// Reuse the model's existing DGELSD provider and row-major staging. Rank
// deficient and underdetermined systems keep the same SVD policy as the
// source's numpy.linalg.lstsq; no normal-equations approximation is used.
inline LeastSquares least_squares(const Matrix& a, const Matrix& b, double rcond=1e-12) {
    validate(a); validate(b);
    if(a.rows!=b.rows || !std::isfinite(rcond) || rcond<0.)
        throw std::invalid_argument("dense least squares: incompatible dimensions/cutoff");
    using Integer=::rider::lapack_int;
    const auto checked=[](std::size_t value) {
        if(std::cmp_greater(value,std::numeric_limits<Integer>::max()))
            throw std::invalid_argument("least squares dimension exceeds provider integer ABI");
        return static_cast<Integer>(value);
    };
    const auto m=checked(a.rows),n=checked(a.cols),nrhs=checked(std::max<std::size_t>(1,b.cols));
    ::rider::LeastSquaresWorkspace workspace(m,n,nrhs);
    const Vector empty_rhs(b.cols==0?a.rows:0,0.);
    auto solved=workspace.solve(a.values,m,n,b.cols==0?std::span<const double>(empty_rhs):std::span<const double>(b.values),nrhs,rcond);
    LeastSquares result{{a.cols,b.cols,b.cols==0?Vector{}:std::move(solved.solution)},static_cast<std::size_t>(solved.rank),std::move(solved.singular_values)};
    if(!finite(result.solution.values)) throw std::runtime_error("dense least squares: nonfinite solution");
    return result;
}
} // namespace bike::numerics::dense
