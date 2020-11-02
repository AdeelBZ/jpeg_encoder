#include <iostream>
// #include "../include/class.h"
#include "jpegCodec.h"
#include "stb_image.h"
#include "stb_image_write.h"
// #include "../include/jpegCodec.h"
#include <thread>



/* function prototypes */
void print_img_data_hex(unsigned char *a_imgData, char *a_file_name, int a_width, int a_height, int a_channels);

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        std::cerr << "You must enter [SOURCE] [DESTINATION]\n";
        exit(EXIT_FAILURE);
    }

    char *l_src = argv[1];
    char *l_dest = argv[2];

    int width, height, channel_num;
    unsigned char *imgData = stbi_load(l_src, &width, &height, &channel_num, 0);

    if (!imgData)
    {
        std::cerr << "Failed to load image. Program Exiting... " << l_src << '\n';
        exit(EXIT_FAILURE);
    }
    else
    {
        std::cout << "Image has width " << width << " height " << height << " And " << channel_num << " channel(s)\n";
    }

    /* write data to image file */

    JPEGCodec testJpeg(8);
    std::cout << "Start jpeg encoding of file: " << l_src << '\n';
    testJpeg.encode(imgData, l_src, l_dest, width, height, channel_num, 91);
    stbi_write_jpg("jpegexample.jpg", width, height, channel_num, imgData, 91);
    std::cout << "Saved jpeg image to: " << l_dest << '\n';

    return 0;
}
