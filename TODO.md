# TO DO
- [ ] multiply fixed size matrices using NEON instructions
  - https://developer.arm.com/documentation/102467/0201/Example---matrix-multiplication
- [ ] add shape and math descriptions to variables
- [ ] There is probably a way to make getSection just return a pointer to the 'top left corner' of the section so it doesn't have to create a new matrix
- [ ] It is probably best to use a premade matrix library because it can use SIMD and other optimizations
  - or I could learn how to use those things but that is a bit of a can of worms
- [ ] if eta_prev or f_prev is ever more than 1x6xnx? it will break the soft sim
- [ ] does matrix add work with 3d matrices?
- [ ] Create python 3d visualization of the rotation matrices
- [ ] Create comments to explain the math behind the functions
- [ ] Add tests (could I use original matlab code to test?)
- [ ] matrix_plusEq(original matrix, othermatrix)
  - add this to make adding something to itself faster and use less memory
  - no malloc needed
- [ ] matrix_minusEq(original matrix, othermatrix)
  - same as pluseq
- [ ] matrix_timesEq(original matrix, othermatrix)
  - same as pluseq
  - also add scalar version


# THINGS TO FIX
- [x] Determinant keeps trying to free null pointers, not sure why
  - fixed, added check for null pointers with print, but I don't know what actually caused it.
- [ ] Probably should make some kind of union for SE3 and matrix or something so I don't have to create a new one every time I want to convert


# OPTIMIZATIONS
- [ ] could use union to combine so3 and se3 etc. not sure if that is faster or slower
- [ ] use `restrict` keyword to tell the compiler that the pointers are the only ref. to the data within the function
  - Is this useful in modern compilers? i.e. will this happen automatically?
- [ ] struct size should be a power of 2 if possible for array of structs
- [ ] for switch statements cases in a small range use a jump table instead of if-elseif ladder
- [ ] declare local vars in the innermost scope possible (this will help in the determinant function for sub I think)
- [ ] it might be faster to pass a pointer to store result in instead of returning a pointer to the result but modern compilers probably fix this automatically 