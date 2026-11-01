#include "RIM_functions.h"
#include <stdio.h>
#include <math.h>

// ==========================================
// ベクトル演算系
// ==========================================

// 2つのベクトルの加算 (C = A + B)
// [引数] N: 要素数, A, B: 入力ベクトル, C: 結果格納用ベクトル
void add_vec(int N, double A[N], double B[N], double C[N])
{
    for (int i = 0; i < N; i++) {
        C[i] = A[i] + B[i];
    }
}

// 2つのベクトルの減算 (C = A - B)
// [引数] N: 要素数, A, B: 入力ベクトル, C: 結果格納用ベクトル
void sub_vec(int N, double A[N], double B[N], double C[N])
{
    for (int i = 0; i < N; i++) {
        C[i] = A[i] - B[i];
    }
}

// ベクトルのスカラー倍 (C = k * A)
// [引数] N: 要素数, A: 入力ベクトル, C: 結果格納用ベクトル, k: 倍率(スカラー)
void scale_vec(int N, double A[N], double C[N], double k)
{
    for (int i = 0; i < N; i++) {
        C[i] = k * A[i];
    }
}

// ベクトルのノルム(大きさ/長さ)の計算 (||A||)
// [引数] N: 要素数, A: 入力ベクトル
// [戻り値] ベクトルのノルム(double)
double norm_vec(int N, double A[N])
{
    double result = 0.0;

    for (int i = 0; i < N; i++) {
        result += A[i] * A[i];
    }

    return sqrt(result);
}

// ベクトルの正規化 (B = A / ||A||)
// [引数] N: 要素数, A: 入力ベクトル, B: 結果格納用(単位ベクトル)
// [戻り値] 成功時: 0, ゼロベクトルのため失敗時: -1
int normalize_vec(int N, double A[N], double B[N])
{
    double norm = norm_vec(N, A);

    if (norm < 1e-9) {
        printf("ゼロベクトルは正規化できません\n");
        return -1;
    }

    for (int i = 0; i < N; i++) {
        B[i] = A[i] / norm;
    }

    return 0;
}

// 2つのベクトルの内積計算 (A · B)
// [引数] N: 要素数, A, B: 入力ベクトル
// [戻り値] 内積の値(double)
double dot_vec(int N, double A[N], double B[N])
{
    double result = 0.0;

    for (int i = 0; i < N; i++) {
        result += A[i] * B[i];
    }

    return result;
}

// 3次元ベクトルの外積(クロス積)計算 (C = A × B)
// [引数] A, B: 3次元の入力ベクトル, C: 結果格納用ベクトル
void cross_vec(double A[3], double B[3], double C[3])
{
    C[0] = A[1] * B[2] - A[2] * B[1];
    C[1] = A[2] * B[0] - A[0] * B[2];
    C[2] = A[0] * B[1] - A[1] * B[0];
}

// ベクトルのコピー (dest = src)
// [引数] N: ベクトルの要素数, src: コピー元ベクトル, dest: コピー先ベクトル
void copy_vec(int N, double src[N], double dest[N])
{
    for (int i = 0; i < N; i++) {
        dest[i] = src[i];
    }
}

// ==========================================
// 行列演算系
// ==========================================

// N×N の単位行列の生成 (A = I)
// [引数] N: 行列の次数, A: 結果格納用正方行列
void eye_mat(int N, double A[N][N])
{
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            A[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
}

// 2つの行列の加算 (C = A + B)
// [引数] row: 行数, col: 列数, A, B: 入力行列, C: 結果格納用行列
void add_mat(int row, int col, double A[row][col], double B[row][col], double C[row][col]){
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}

// 2つの行列の減算 (C = A - B)
// [引数] row: 行数, col: 列数, A, B: 入力行列, C: 結果格納用行列
void sub_mat(int row, int col,double A[row][col],double B[row][col],double C[row][col]){
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            C[i][j] = A[i][j] - B[i][j];
        }
    }
}

// 行列とベクトルの積 (y = A * x)
// [引数] row: 行数, col: 列数, A: 入力行列, x: 入力ベクトル(要素数col), y: 結果格納用ベクトル(要素数row)
void mat_vec_mul(int row, int col,double A[row][col],double x[col],double y[row]){
    for (int i = 0; i < row; i++) {
        y[i] = 0.0;

        for (int j = 0; j < col; j++) {
            y[i] += A[i][j] * x[j];
        }
    }
}

// 2つの行列の積 (C = A * B)
// [引数] rowA, colA: 行列Aのサイズ, rowB, colB: 行列Bのサイズ, A, B: 入力行列, C: 結果格納用行列
// [戻り値] 成功時: 0, サイズ不一致による失敗時: -1
int mul_mat(int rowA, int colA,int rowB, int colB,double A[rowA][colA],double B[rowB][colB],double C[rowA][colB])
{
    if (colA != rowB) {
        printf("行列のサイズが合いません\n");
        return -1;
    }

    for (int i = 0; i < rowA; i++) {
        for (int j = 0; j < colB; j++) {
            C[i][j] = 0;

            for (int k = 0; k < colA; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return 0;
}

// 行列のコピー (dest = src)
// [引数] row, col: 行列の行数および列数, src: コピー元行列, dest: コピー先行列
void copy_mat(int row, int col, double src[row][col], double dest[row][col])
{
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            dest[i][j] = src[i][j];
        }
    }
}

// 行列の転置 (B = A^T)
// [引数] row: Aの行数, col: Aの列数, A: 入力行列(row×col), B: 結果格納用行列(col×row)
void transpose_mat(int row, int col,double A[row][col],double B[col][row])
{
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            B[j][i] = A[i][j];
        }
    }
}

// 行列のスカラー倍 (C = k * A)
// [引数] rows: 行数,cols: 列数, A: 入力行列,C: 結果格納用行列, k: 倍率（スカラー）
void scale_mat(int rows,int cols,double A[rows][cols],double C[rows][cols],double k)
{
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            C[i][j] = k * A[i][j];
        }
    }
}

// 逆行列の計算 (inv = A^-1) [ガウス・ジョルダン法]
// [引数] N: 次数, A: 入力正方行列, inv: 結果格納用逆行列
// [戻り値] 成功時: 0, 逆行列が存在しない(正則でない)場合: -1
//参考URL:https://imagingsolution.net/math/gauss-jordan-with-pivot/
int inv_mat(int N, double A[N][N], double inv[N][N])
{
    double temp[N][N];

    // temp = A
    // inv = 単位行列
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            temp[i][j] = A[i][j];
            inv[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    for (int i = 0; i < N; i++) {

        // ピボット選択
        int max_row = i;

        for (int k = i + 1; k < N; k++) {
            if (fabs(temp[k][i]) > fabs(temp[max_row][i])) {
                max_row = k;
            }
        }

        // 行交換
        if (max_row != i) {
            for (int j = 0; j < N; j++) {
                double t = temp[i][j];
                temp[i][j] = temp[max_row][j];
                temp[max_row][j] = t;

                t = inv[i][j];
                inv[i][j] = inv[max_row][j];
                inv[max_row][j] = t;
            }
        }

        // 逆行列が存在しない
        if (fabs(temp[i][i]) < 1e-14) {
            printf("逆行列が存在しません\n");
            return -1;
        }

        // ピボットを1にする
        double pivot = temp[i][i];

        for (int j = 0; j < N; j++) {
            temp[i][j] /= pivot;
            inv[i][j] /= pivot;
        }

        // 他の行を0にする
        for (int k = 0; k < N; k++) {
            if (k == i) {
                continue;
            }

            double factor = temp[k][i];

            for (int j = 0; j < N; j++) {
                temp[k][j] -= factor * temp[i][j];
                inv[k][j] -= factor * inv[i][j];
            }
        }
    }

    return 0;
}







