add_library(se3_linalg INTERFACE
        vector_concepts.hpp
        vectors.hpp
        vector_ops.hpp
        matrices.hpp
    transpose.hpp
    matrix_ops.hpp
    simd/matgroup_simd.hpp
)
add_library(se3::linalg ALIAS se3_linalg)