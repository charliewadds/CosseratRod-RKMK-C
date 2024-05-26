# TO DO
- [ ] Array sizes are written as static right now because it helped me with the math. they should be dynamic and I will change that soon
- [ ] The robot struct is a nightmare and needs to be rewritten. It is possibly the worst code I have ever written.
- [ ] I think it would be faster to have global temporary matrices for functions to use for math instead of mallocing and freeing them every time.
  - this would be a nightmare if I end up using concurrency though
- [x] I think getting rid of SO3 and SE3 is going to be best in the long run
- [x] one joint is upside down or someting in the rigid Kin
- [ ] multiply fixed size matrices using NEON instructions
  - https://developer.arm.com/documentation/102467/0201/Example---matrix-multiplication
- [ ] add shape and math descriptions to variables
- [x] There is probably a way to make getSection just return a pointer to the 'top left corner' of the section so it doesn't have to create a new matrix
- [ ] It is probably best to use a premade matrix library because it can use SIMD and other optimizations
  - or I could learn how to use those things but that is a bit of a can of worms
- [ ] if eta_prev or f_prev is ever more than 1x6xnx? it will break the soft sim
- [ ] does matrix add work with 3d matrices?
  - no, this needs to be implemented
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


- laurine chalmers for access
# THINGS TO FIX
- [x] Determinant keeps trying to free null pointers, not sure why
  - fixed, added check for null pointers with print, but I don't know what actually caused it.
- [ ] Probably should make some kind of union for SE3 and matrix or something so I don't have to create a new one every time I want to convert


# OPTIMIZATIONS
- [ ] speed up root finding.
  - an analytical derivative for the BCS would, I think, be much faster than finite differences.
  - should plot the error to see how accurate finite differences are
  - if no analytical derivative is possible, it might be faster to parallelize the finite differences. Although there is some overhead to this so it might not be worth it.
     pipelining might reduce the overhead a bit
- [ ] I think it would be faster to use contiguously allocated matrices which expand like java arraylists
  - use realloc (I always forget it exists)
  - this would let me use non-size constrained matrices and therefore less temporary variables
  - they should probably start as 6x6 because I think the only thing bigger are the eta and f prev
- [ ] similar to above, preallocating memory where possible could speed things up in the functions that are run many times
- [ ] could use union to combine so3 and se3 etc. not sure if that is faster or slower
- [ ] use `restrict` keyword to tell the compiler that the pointers are the only ref. to the data within the function
  - Is this useful in modern compilers? i.e. will this happen automatically?
- [ ] struct size should be a power of 2 if possible for array of structs
- [ ] for switch statements cases in a small range use a jump table instead of if-elseif ladder
- [ ] declare local vars in the innermost scope possible (this will help in the determinant function for sub I think)
- [ ] it might be faster to pass a pointer to store result in instead of returning a pointer to the result but modern compilers probably fix this automatically 