#define LEMON_VERSION "1.3.1"
#define LEMON_HAVE_LONG_LONG 1

/* #undef LEMON_HAVE_LP */
/* #undef LEMON_HAVE_MIP */
/* #undef LEMON_HAVE_GLPK */
/* #undef LEMON_HAVE_CPLEX */
/* #undef LEMON_HAVE_SOPLEX */
/* #undef LEMON_HAVE_CLP */
/* #undef LEMON_HAVE_CBC */

#define _LEMON_CPLEX 1
#define _LEMON_CLP 2
#define _LEMON_GLPK 3
#define _LEMON_SOPLEX 4
#define _LEMON_CBC 5

/* #undef LEMON_DEFAULT_LP */
/* #undef LEMON_DEFAULT_MIP */

/* #undef LEMON_USE_PTHREAD */
//#define LEMON_USE_WIN32_THREADS 1

#ifdef WIN32
#define LEMON_WIN32
#else
#define LEMON_WIN32 // TODO: comment it out on non-windows platform.
#endif // WIN32
