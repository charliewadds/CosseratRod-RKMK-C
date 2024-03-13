# THINGS TO FIX
- [ ] Determinant keeps trying to free null pointers, not sure why


# OPTIMIZATIONS
- [ ] could use union to combine so3 and se3 etc. not sure if that is faster or slower
- [ ] use `restrict` keyword to tell the compiler that the pointers are the only ref. to the data within the function
- [ ] struct size should be a power of 2 if possible for array of structs
- [ ] for switch statements cases in a small range use a jump table instead of if-elseif ladder
- [ ] declare local vars in the innermost scope possible (this will help in the determinant function for sub I think)
- [ ] it might be faster to pass a pointer to store result in instead of returning a pointer to the result but modern compilers probably fix this automatically 