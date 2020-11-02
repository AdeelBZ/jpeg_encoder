// #pragma once
#ifndef CODEC_H
#define CODEC_H
#include <iostream>
#include <fstream>


class Codec
{
protected:
    /* input file for reading and decoding the data */
    std::ifstream inputFile;
    /* output file for writing and encoding the data */
    std::ofstream m_output_encoded_file;

public:
    virtual void encode(unsigned char *a_imgData, char *a_file_name, char *a_dest, int a_width, int a_height, int a_channels) = 0;
    virtual void encode(unsigned char *a_imgData, char *a_file_name, char *a_dest, int a_width, int a_height, int a_channels, int a_quality) = 0;

    void decode();

protected:
    /* Got to do this per element as otherwise you will insert as a character which is not in its binary format */
    template <class T>
    void write_byte_in_file(T *a_arr, size_t a_arr_size = 1)
    {
        for (int i = 0; i < a_arr_size; i++)
        {
            m_output_encoded_file.put(a_arr[i]); // array must be written byte-byte or there will be inaccuracies  if done another way
        }
    }
};

#endif