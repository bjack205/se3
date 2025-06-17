add_library(se3_linalg INTERFACE
        vector_concepts.hpp
        vectors.hpp
        vector_ops.hpp
        matrices.hpp
)
add_library(se3::linalg ALIAS se3_linalg)