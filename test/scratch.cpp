#include <type_traits> // For std::conditional_t, std::decay_t, std::is_lvalue_reference_v
#include <utility>     // For std::forward, std::move
#include <iostream>    // For std::cout

// --- Simple Matrix Class for Demonstration ---
struct Matrix {
    int data[4]; // Example data
    Matrix() {
        std::cout << "Matrix default constructed\n";
        for(int i=0; i<4; ++i) data[i] = 0;
    }
    Matrix(const Matrix& other) {
        std::cout << "Matrix copy constructed\n";
        for (int i = 0; i < 4; ++i) data[i] = other.data[i];
    }
    Matrix(Matrix&& other) noexcept {
        std::cout << "Matrix move constructed\n";
        for (int i = 0; i < 4; ++i) data[i] = other.data[i];
        for (int i = 0; i < 4; ++i) other.data[i] = 0; // Clear source for demo
    }
    Matrix& operator=(const Matrix& other) {
        std::cout << "Matrix copy assigned\n";
        if (this != &other) {
            for (int i = 0; i < 4; ++i) data[i] = other.data[i];
        }
        return *this;
    }
    Matrix& operator=(Matrix&& other) noexcept {
        std::cout << "Matrix move assigned\n";
        if (this != &other) {
            for (int i = 0; i < 4; ++i) data[i] = other.data[i];
        }
        return *this;
    }
    ~Matrix() {
        std::cout << "Matrix destructed\n";
    }

    Matrix operator*(const Matrix& other) const {
        Matrix result;
        std::cout << "Matrix multiplication (returning new Matrix)\n";
        result.data[0] = data[0] * 10 + other.data[0]; // Dummy operation
        return result;
    }

    void print() const {
        std::cout << "Matrix Data: [" << data[0] << ", " << data[1] << ", " << data[2] << ", " << data[3] << "]\n";
    }
};

// --- Transpose Class ---
// This class stores either a reference to the parent matrix (for l-values)
// or owns a copy/moved version of the parent matrix (for r-values/temporaries).

// The class template parameter T_MatrixType will be the *decayed* type (e.g., Matrix),
// because the constructor's template parameter will handle the reference-ness.
template <typename T_MatrixType> // T_MatrixType will be the base type (e.g., Matrix)
class Transpose {
public:
    // The storage type for the parent matrix.
    // This will either be T_MatrixType& (for references) or T_MatrixType (for values).
    // This determination will be made based on the type U passed to the constructor.
    // We store T_MatrixType directly if it's a value, and T_MatrixType& if it's a reference.

    // A type that can hold either a reference or a value.
    // This relies on the constructor to properly initialize `stored_matrix_`
    // with either a reference to the original, or a moved/copied value.
    // We cannot simply use std::conditional_t directly on T_MatrixType for `stored_matrix_`,
    // because T_MatrixType is always the non-reference type (e.g., Matrix).
    // The reference-ness comes from the constructor's universal reference.

    // To allow `Transpose(lvalue)` and `Transpose(rvalue)` to work and deduce `auto`:
    // The `Transpose` class needs a way to store *either* `T_MatrixType&` or `T_MatrixType`.
    // The previous approach implicitly relied on the class template parameter
    // being exactly what was deduced by the constructor.
    //
    // The cleanest way is to make the constructor itself a template and use
    // SFINAE or `std::is_constructible` to constrain it,
    // or simply rely on the perfect forwarding with a flexible internal type.

    // The internal storage type: it will be a value if it's a temporary,
    // and a reference if it's an l-value.
    // We need to use `std::variant` or a custom union, or just a `std::reference_wrapper`
    // for l-values and a `T_MatrixType` for r-values.

    // A simpler approach for the storage itself:
    // We *always* store `T_MatrixType` (the decayed type) as a value.
    // If constructed with an l-value, it will copy. If with an r-value, it will move.
    // This is not what you wanted ("store a reference ... if l-value").

    // To store a *reference* for l-values and *value* for r-values,
    // we need to make the class template parameter `T_MatrixType` itself
    // reflect the reference-ness, and then use `std::conditional_t` based on that.
    // This means the class deduction guide is crucial.

    // Re-thinking again for simplicity and correct behavior with auto-deduction
    // and reference storage for l-values without a custom wrapper struct.
    // This is typically handled by having a template parameter on the class
    // that *is* the actual type including reference/constness.

    // This is the classic way to do it:
    // The template parameter `T` of the class `Transpose<T>` *is* the type of the argument
    // received by the forwarding constructor.

    // The internal storage type.
    // If T is Matrix&, StorageType is Matrix&.
    // If T is Matrix, StorageType is Matrix. (From Matrix&&, it decays to Matrix)
    using storage_type = std::conditional_t<
        std::is_lvalue_reference_v<T_MatrixType>,
        T_MatrixType,
        std::decay_t<T_MatrixType>
    >;

    storage_type stored_matrix_;

public:
    // Constructor uses a DIFFERENT template parameter `U` for the forwarding reference.
    // This allows `U` to deduce as `Matrix&`, `const Matrix&`, or `Matrix`.
    // Then, `T_MatrixType` of the class itself is deduced from `U`.
    template <typename U>
    Transpose(U&& m) : stored_matrix_(std::forward<U>(m)) {
        std::cout << "Transpose object created (auto-deduced).\n";
        // Diagnostic prints:
        // std::cout << "  Deduced U type: " << typeid(U).name() << "\n";
        // std::cout << "  Class T_MatrixType: " << typeid(T_MatrixType).name() << "\n";
        // std::cout << "  Actual stored_matrix_ type: " << typeid(stored_matrix_).name() << "\n";
    }

    // Accessor for the underlying matrix.
    // decltype(auto) ensures the correct reference-ness is returned.
    decltype(auto) get_matrix() {
        return stored_matrix_;
    }

    decltype(auto) get_matrix() const {
        return stored_matrix_;
    }

    // --- Example Transpose Operations ---
    Matrix multiply_by_vector(int v) const {
        std::cout << "Performing transpose multiplication with vector...\n";
        Matrix result;
        result.data[0] = get_matrix().data[0] * v;
        return result;
    }

    Matrix operator*(const Matrix& other) const {
        std::cout << "Performing transpose multiplication with another matrix...\n";
        Matrix result;
        result.data[0] = get_matrix().data[0] + other.data[0]; // Dummy operation
        return result;
    }
};

// --- C++17 Class Template Argument Deduction (CTAD) Guide ---
// This is the magic that makes `auto t = Transpose(m);` work for l-values and r-values
// while `Transpose` itself is templated on the *actual type that was passed*.
template <typename U>
Transpose(U&&) -> Transpose<U>;
// Explanation: When `Transpose(U_arg)` is called, this guide tells the compiler
// to deduce the class template parameter `T_MatrixType` as `U` (the type of `U_arg`
// including its reference/constness), not its decayed form.

int main() {
    Matrix m0; m0.data[0] = 5;
    Matrix m1; m1.data[0] = 7;

    std::cout << "\n--- Case 1: Transpose of an l-value (m0) ---\n";
    // `U` will be `Matrix&`. The CTAD guide will make `T_MatrixType` be `Matrix&`.
    // `stored_matrix_` will be `Matrix&`.
    auto m0_transpose = Transpose(m0);
    std::cout << "m0.data[0] before modification: " << m0.data[0] << "\n";
    m0_transpose.get_matrix().data[0] = 99; // Modifies original m0
    std::cout << "m0.data[0] after modifying through transpose: " << m0.data[0] << "\n";
    m0_transpose.multiply_by_vector(2).print();


    std::cout << "\n--- Case 2: Transpose of an r-value (m0 * m1) ---\n";
    // `U` will be `Matrix` (from temporary). The CTAD guide will make `T_MatrixType` be `Matrix`.
    // `stored_matrix_` will be `Matrix`.
    std::cout << "\nAbout to create Transpose(m0 * m1)\n";
    auto m01_transpose = Transpose(m0 * m1); // m0 * m1 is a temporary (r-value)
    std::cout << "Transpose(m0 * m1) created.\n";
    m01_transpose.multiply_by_vector(3).print();

    // m01_transpose.get_matrix().data[0] = 500;
    std::cout << "m01_transpose internal data after modification: ";
    m01_transpose.get_matrix().print();
    std::cout << "m0.data[0] (unaffected): " << m0.data[0] << "\n";


    std::cout << "\n--- Case 3: Chained operations (Transpose(m0 * m1) * v) ---\n";
    std::cout << "\nAbout to perform Transpose(m0 * m1) * Matrix()\n";
    Matrix result_chained = Transpose(m0 * m1) * Matrix();
    std::cout << "Result of chained operation: ";
    result_chained.print();


    std::cout << "\n--- Case 4: Transpose of a const l-value (cm) ---\n";
    const Matrix cm;
    // `U` will be `const Matrix&`. The CTAD guide will make `T_MatrixType` be `const Matrix&`.
    // `stored_matrix_` will be `const Matrix&`.
    std::cout << "\nAbout to create Transpose(cm)\n";
    auto cm_transpose = Transpose(cm);
    std::cout << "cm.data[0]: " << cm.data[0] << "\n";
    std::cout << "cm_transpose internal data: ";
    cm_transpose.get_matrix().print();
    // cm_transpose.get_matrix().data[0] = 1; // This would be a compile error, as expected (const reference)

    std::cout << "\n--- End of main ---\n";
    return 0;
}