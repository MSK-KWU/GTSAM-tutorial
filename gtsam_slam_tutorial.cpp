#include "gtsam_factor_graph.hpp"
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Matrix.h>

using namespace gtsam;
using namespace std;
// #define MATRIX_SIZE 50

int main()
{
    Matrix matrix_23(2, 3);

    //these are same 3 x 1 vector
    Vector3 vd_3d;
    Vector v_3d(3);
    

    //insert data
    matrix_23 << 1, 2, 3, 4, 5, 6;
    cout << "matrix 2x3 from 1 to 6: \n" << matrix_23 << endl;
    
    //check
    cout << "\nprint matrix 3x2: " << endl;
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++)
        cout << matrix_23(i, j) << "\t";
        cout << endl;
    }


    //using vector for some operations..?
    vd_3d << 1, 3, 5;
    v_3d << 2, 4, 6;
    
    //check
    cout << vd_3d << endl;
    cout << v_3d << endl;

    //convert matrix into vectors, cuz GTSAM follows the Eigen.
    //Eigen convert the datatype but GTSAM..?
    Matrix result = matrix_23 * v_3d;
    cout << "Matrix vector multiplication : [" << result.transpose() << "]" << endl;

    return 0;
}
