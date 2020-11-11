#pragma once

namespace fastsense::registration
{

/*Vector6f xi;
Type A[N][N];
Type b[N];
Type x[N];

for (int i = 0; i < N; i++)
{
    for (int j = 0 ; j < N; j++)
    {
        A[i][j] = hf(i, j);
    }
    b[i] = -gf(i);
}

lu_decomposition(A);
lu_solve(A, b, x);

for (int i = 0; i < N; i++)
{
    xi(i) = x[i];
}*/

using Type = double;

constexpr int N = 6;

void lu_decomposition(Type A[N][N])
{
    for (int i = 0; i < N - 1; i++)
    {
        for (int k = i + 1; k < N; k++)
        {
            A[k][i] /= A[i][i];
            for (int j = i + 1; j < N; j++)
            {
                A[k][j] -= A[k][i] * A[i][j];
            }
        }
    }
}

void lu_solve(Type A[N][N], Type b[N], Type x[N])
{
    for (int i = 0; i < N; i++)
    {
        x[i] = b[i];
        for (int k = 0; k < i; k++)
        {
            x[i] -= A[i][k] * x[k];
        }
    }

    for (int i = N - 1; i >= 0; i--)
    {
        for (int k = i + 1; k < N; k++)
        {
            x[i] -= A[i][k] * x[k];
        }
        x[i] /= A[i][i];
    }
}

void lu_decomposition(Type A[N][N], Type L[N][N], Type R[N][N])
{
    for (int i = 0; i < N; i++)
    {
        for (int j = 0 ; j < N; j++)
        {
            R[i][j] = A[i][j];
            L[i][j] = i == j ? 1 : 0;
        }
    }

    for (int i = 0; i < N - 1; i++)
    {
        for (int k = i + 1; k < N; k++)
        {
            L[k][i] = R[k][i] / R[i][i];
            for (int j = i; j < N; j++)
            {
                R[k][j] -= L[k][i] * R[i][j];
            }
        }
    }
}

}