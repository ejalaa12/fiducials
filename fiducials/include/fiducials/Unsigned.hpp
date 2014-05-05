// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(UNSIGNED_H_INCLUDE)
#define UNSIGNED_H_INCLUDE 1


/// @brief *Unsigned* is a 32-bit unsigned integer.
typedef unsigned int Unsigned;

// *Unsigned* routines:

extern int Unsigned__compare(Unsigned unsigned1, Unsigned unsigned2);
extern bool Unsigned__equal(Unsigned unsigned1, Unsigned unsigned2);
extern Unsigned Unsigned__hash(Unsigned unsigned1);
extern Unsigned Unsigned__minimum(Unsigned unsigned1, Unsigned unsigned2);

#endif // !defined(UNSIGNED_H_INCLUDE)