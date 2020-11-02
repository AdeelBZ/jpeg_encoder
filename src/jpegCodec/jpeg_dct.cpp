#include "../../include/jpegCodec.h"
#include <cmath> //for std::sqrt


void JPEGCodec::calculate_c_terms(int a_square_size)
{

    m_cosine_terms = new double[2]; // you only need to as k only results in 2 different c_terms, one for when k = 0 and another when 1 <= k <= a_square_size
    if (!m_cosine_terms)
    {
        std::cerr << "Failed to dynamically allcoate memory for the cosine terms";
        exit(EXIT_FAILURE);
    }

    for (int k = 0; k < 2; k++)
    {
        if (k == 0)
        {
            m_cosine_terms[0] = std::sqrt((1.0 / a_square_size)); // we change the multiplier when calculating a cosine only when k == 0
            // c_terms[0] = (1.0 / std::sqrt(2.0)) * std::sqrt((2.0 / a_square_size)); // another way of calculating the same value
        }
        else
        {
            m_cosine_terms[1] = std::sqrt((2.0 / a_square_size)); // when k> 0
        }
    }
}

void JPEGCodec::calculate_cosines(int a_square_size)
{
    m_cosines = new double *[a_square_size]; /* establish rows */
    if (!m_cosines)
    {
        std::cerr << "Failed to dynamically allcoate memory for cosines";
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < a_square_size; i++)
    {
        m_cosines[i] = new double[a_square_size];
        if (!m_cosines[i])
        {
            std::cerr << "Could not dynamically allcoate row for cosines";
            exit(EXIT_FAILURE);
        }
    }

    /* perform cosine calculation */
    for (int col = 0; col < a_square_size; col++)
    {
        for (int row = 0; row < a_square_size; row++)
        {
            double numerator = (M_PI * col) * (2.0 * row + 1);
            double denominator = 2 * a_square_size;
            double cos_res = std::cos(numerator / denominator); /* std::cos expects radians, not degrees */
            m_cosines[row][col] = cos_res;
            /* we're sorting each cosine in retropsect to columns so:
                    column 0 is for k = 0
                    column 1 is for k = 1
                    column 2 is for k = 2
                etc
                */
        }
    }
}

/* specfiically, we are looking at 2D - DCT. Keep this and don't use AAN for now*/
void JPEGCodec::compute_DCT(float *dct_pixel_input, int a_square_size)
{
    float DCT[a_square_size][a_square_size];

    int i, j, x, y;
    double temp;
    for (i = 0; i < a_square_size; i++)
    {
        for (j = 0; j < a_square_size; j++)
        {
            int pos = 0;
            temp = 0.0f;
            for (x = 0; x < a_square_size; x++)
            {
                for (y = 0; y < a_square_size; y++, pos++)
                {
                    temp += m_cosines[x][i] *
                            m_cosines[y][j] *
                            dct_pixel_input[pos];
                }
            }
            double j_c_term = (j == 0) ? m_cosine_terms[0] : m_cosine_terms[1];
            double i_c_term = (i == 0) ? m_cosine_terms[0] : m_cosine_terms[1];

            temp *= i_c_term * j_c_term;
            // DCT[i][j] = std::round(temp);
            DCT[i][j] = temp;
        }
    }

    int pos = 0;
    for (int i = 0; i < a_square_size; i++)
    {
        for (int j = 0; j < a_square_size; j++, pos++)
        {
            dct_pixel_input[pos] = DCT[i][j];
        }
    }
}
