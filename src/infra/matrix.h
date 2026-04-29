#ifndef _matrix_h_
#define _matrix_h_



// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 矩阵错误码
 * @param MATRIX_SUCCESS 成功
 * @param MATRIX_ERROR 一般错误
 * @param MATRIX_CREATE_FAILED 矩阵创建失败，可能是内存不足
 * @param MATRIX_INVALID 无效的矩阵输入，例如空指针或维度不匹配
 * @param MATRIX_CANNOT_COMPUTE 无法计算，例如矩阵乘法维度
 * @param MATRIX_PIVOT_IS_ZERO 在求逆过程中遇到零主元，矩阵可能是奇异的
 * @param MATRIX_INPLACE 输入输出矩阵指向同一内存地址，可能导致未定义行为
 */
typedef enum {
    MATRIX_SUCCESS = 0,
    MATRIX_ERROR,
    MATRIX_CREATE_FAILED,
    MATRIX_INVALID,
    MATRIX_CANNOT_COMPUTE,
    MATRIX_PIVOT_IS_ZERO,
    MATRIX_INPLACE,
} MatrixErrorCode;

/**
 * @brief 矩阵结构体，包含数据指针和维度信息
 * @param pdata 指向矩阵数据的指针，按行优先存储
 * @param row 矩阵的行数
 * @param col 矩阵的列数
 */
typedef struct {
    float* pdata;
    unsigned int row;
    unsigned int col;
} Matrix;

/**
 * @brief 3D 向量结构体，包含 x、y、z 三个分量
 * @param x 向量的 x 分量
 * @param y 向量的 y 分量
 * @param z 向量的 z 分量
 */
typedef struct {
    float x;
    float y;
    float z;
} Vector3;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 创建一个矩阵，用户需要提供数据缓冲区
 * @param name 矩阵名称
 * @param row 行数
 * @param col 列数
 * @note 该宏会定义一个名为 name##_data 的 float 数组作为矩阵数据存储，并创建一个 Matrix 结构体指向该数据
 * @note 创建后 name 就是可用的矩阵变量
 */
#define matrix_create(name, row, col) \
    static float name##_data[row * col]; \
    Matrix name; \
    matrix(&name, row, col, name##_data)

/**
 * @brief 创建一个单位矩阵，用户需要提供数据缓冲区
 * @param name 矩阵名称
 * @param size 矩阵的行列数，单位矩阵必须是方阵
 * @note 该宏会定义一个名为 name##_data 的 float 数组作为矩阵数据存储，并创建一个 Matrix 结构体指向该数据，然后将其初始化为单位矩阵
 * @note 创建后 name 就是可用的单位矩阵变量
 */
#define matrix_identity_create(name, size) \
    static float name##_data[size * size]; \
    Matrix name; \
    matrix_identity(&name, size, name##_data)

/**
 * @brief 创建一个矩阵
 * @param m 指向 Matrix 结构的指针
 * @param row 矩阵的行数
 * @param col 矩阵的列数
 * @param data 矩阵数据缓冲区指针 (按行优先存储)
 * @return 操作状态
 */
MatrixErrorCode matrix(Matrix* const m, unsigned int row, unsigned int col, float* data);
/**
 * @brief 创建一个单位矩阵
 * @param m 指向 Matrix 结构的指针
 * @param size 矩阵的行列数 (必须为方阵)
 * @param data 矩阵数据缓冲区指针
 * @return 操作状态
 */
MatrixErrorCode matrix_identity(Matrix* const m, unsigned int size, float* data);
/**
 * @brief 获取矩阵中指定位置的元素
 * @param m 指向源矩阵的指针
 * @param r 行索引 (0-based)
 * @param c 列索引 (0-based)
 * @param value 指向存储元素值的变量指针
 * @return 操作状态
 */
MatrixErrorCode matrix_get(const Matrix* const m, unsigned int r, unsigned int c, float* value);
/**
 * @brief 设置矩阵中指定位置的元素
 * @param m 指向目标矩阵的指针
 * @param r 行索引 (0-based)
 * @param c 列索引 (0-based)
 * @param value 要设置的元素值
 * @return 操作状态
 */
MatrixErrorCode matrix_set(Matrix* const m, unsigned int r, unsigned int c, float value);
/**
 * @brief 复制矩阵
 * @param m 指向源矩阵的指针
 * @param out 指向输出矩阵的指针
 * @return 操作状态
 */
MatrixErrorCode matrix_copy(const Matrix* const m, Matrix* const out);

/**
 * @brief 矩阵加法 (A + B)
 * @param A 指向第一个矩阵的指针
 * @param B 指向第二个矩阵的指针
 * @param out 指向输出矩阵的指针
 * @return 操作状态
 */
MatrixErrorCode matrix_add(const Matrix* const A, const Matrix* const B, Matrix* const out);
/**
 * @brief 矩阵减法 (A - B)
 * @param A 指向第一个矩阵的指针
 * @param B 指向第二个矩阵的指针
 * @param out 指向输出矩阵的指针
 * @return 操作状态
 */
MatrixErrorCode matrix_sub(const Matrix* const A, const Matrix* const B, Matrix* const out);
/**
 * @brief 矩阵标量乘法 (m * scalar)
 * @param m 指向矩阵的指针
 * @param scalar 标量值
 * @param out 指向输出矩阵的指针
 * @return 操作状态
 */
MatrixErrorCode matrix_scalar_mul(const Matrix* const m, float scalar, Matrix* const out);
/**
 * @brief 矩阵乘法 (A * B)
 * @param A 指向第一个矩阵的指针
 * @param B 指向第二个矩阵的指针
 * @param out 指向输出矩阵的指针
 * @return 操作状态
 */
MatrixErrorCode matrix_mul(const Matrix* const A, const Matrix* const B, Matrix* const out);
/**
 * @brief 矩阵转置 (m^T)
 * @param m 指向源矩阵的指针
 * @param out 指向输出矩阵的指针
 * @return 操作状态
 */
MatrixErrorCode matrix_transpose(const Matrix* const m, Matrix* const out);
/**
 * @brief 将矩阵转换为上三角矩阵
 * @param m 指向源矩阵的指针
 * @param out 指向输出矩阵的指针
 * @return 操作状态
 */
MatrixErrorCode matrix_to_upper_triangular(const Matrix* const m, Matrix* const out);
/**
 * @brief 将矩阵转换为下三角矩阵
 * @param m 指向源矩阵的指针
 * @param out 指向输出矩阵的指针
 * @return 操作状态
 */
MatrixErrorCode matrix_to_lower_triangular(const Matrix* const m, Matrix* const out);
/**
 * @brief 计算矩阵行列式
 * @param m 指向矩阵的指针
 * @param out 指向存储行列式值的变量指针
 * @return 操作状态
 */
MatrixErrorCode matrix_determinant(const Matrix* const m, float* out);
/**
 * @brief 计算矩阵逆矩阵 (m^-1)
 * @param m 指向源矩阵的指针
 * @param out 指向输出矩阵的指针
 * @return 操作状态
 */
MatrixErrorCode matrix_inverse(const Matrix* const m, Matrix* const out);

/**
 * @brief 规范化四元数 (单位四元数)
 * @param quat 输入四元数数组 [w, x, y, z]
 * @param out 输出四元数数组 (规范化后)
 * @return 操作状态
 */
MatrixErrorCode quat_normalize(const float quat[4], float out[4]);
/**
 * @brief 将旋转矩阵转换为四元数
 * @param R 指向旋转矩阵的指针
 * @param quat 输出四元数数组 [w, x, y, z]
 * @return 操作状态
 */
MatrixErrorCode matrix_to_quat(const Matrix* const R, float quat[4]);
/**
 * @brief 将四元数转换为旋转矩阵
 * @param quat 输入四元数数组 [w, x, y, z]
 * @param R 指向输出旋转矩阵的指针
 * @return 操作状态
 */
MatrixErrorCode quat_to_matrix(const float quat[4], Matrix* const R);

/**
 * @brief 3D 向量加法 (a + b)
 * @param a 指向第一个向量的指针
 * @param b 指向第二个向量的指针
 * @param out 指向输出向量的指针
 * @return 操作状态
 */
MatrixErrorCode vec3_add(const Vector3* const a, const Vector3* const b, Vector3* out);
/**
 * @brief 3D 向量减法 (a - b)
 * @param a 指向第一个向量的指针
 * @param b 指向第二个向量的指针
 * @param out 指向输出向量的指针
 * @return 操作状态
 */
MatrixErrorCode vec3_sub(const Vector3* const a, const Vector3* const b, Vector3* out);
/**
 * @brief 3D 向量标量乘法 (v * scalar)
 * @param v 指向向量的指针
 * @param scalar 标量值
 * @param out 指向输出向量的指针
 * @return 操作状态
 */
MatrixErrorCode vec3_scalar_mul(const Vector3* const v, float scalar, Vector3* out);
/**
 * @brief 计算 3D 向量的模长
 * @param v 指向向量的指针
 * @param out 指向存储模长的变量指针
 * @return 操作状态
 */
MatrixErrorCode vec3_norm(const Vector3* const v, float* out);
/**
 * @brief 规范化 3D 向量 (单位向量)
 * @param v 指向源向量的指针
 * @param out 指向输出向量的指针
 * @return 操作状态
 */
MatrixErrorCode vec3_normalize(const Vector3* const v, Vector3* out);
/**
 * @brief 计算两个 3D 向量的点积
 * @param a 指向第一个向量的指针
 * @param b 指向第二个向量的指针
 * @param out 指向存储点积结果的变量指针
 * @return 操作状态
 */
MatrixErrorCode vec3_dot(const Vector3* const a, const Vector3* const b, float* out);
/**
 * @brief 计算两个 3D 向量的叉积 (外积)
 * @param a 指向第一个向量的指针
 * @param b 指向第二个向量的指针
 * @param out 指向输出向量的指针
 * @return 操作状态
 */
MatrixErrorCode vec3_cross(const Vector3* const a, const Vector3* const b, Vector3* out);

#endif
