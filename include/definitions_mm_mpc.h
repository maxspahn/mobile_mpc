#define NC          2   /* Number of config parameters. */
#define NX          10  /* Number of differential state variables.  */
#define NS          1   /* Number of slack variables. */
#define NW          6   /* Number of weights. */
#define NU          10  /* Number of control inputs. */
#define NUF         9   /* Number of control inputs for forces. */
#define NO          0   /* Number of obstacles. */
#define SO          4   /* Size of obstacle data. */
#define NPLANES     0   /* Number of planes. */
#define SPLANES     9   /* Size of plane data. */
#define NINFPLA     60   /* Number of planes. */
#define SINFPLA     4   /* Size of plane data. */
#define NIPES       15  /* Number of infPlanes for each spheres. */
/* Number of online data values.
 * NC + NX + NW + NO * SO + NPLANES * SPLANES + NINFPLA * SINFPLA*/
#define NP          258
#define NPF         260  /* Number of forces parameters. NP + 2 */
#define N           21  /* Number of intervals in the horizon. */
#define TH          20  /* Time horizon. */
