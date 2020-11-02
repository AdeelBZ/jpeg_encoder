#include "../../include/jpegCodec.h"

#include <algorithm> //for std::clamp

/* works */
void JPEGCodec::set_JIFF_APP0_header()
{
    /* Set APP0 */
    TByte l_APP0[] = {JFIFMarkers::START_SECTION, JFIFMarkers::APP0_MARKER};
    this->write_byte_in_file<TByte>(l_APP0, sizeof(l_APP0) / sizeof(l_APP0[0]));

    /* Set length of this header after SOI and APP0. Specifically, set the length of the APP0 field*/
    size_t l_length_of_APP0 = 16; // 16 bytes (14 bytes payload + 2 bytes for this length field)
    this->write_marker_byte_length_in_file(l_length_of_APP0);

    /* Set JFIF */
    TByte l_JIFF_ID[] = {'J', 'F', 'I', 'F', 0}; // JFIF identifier, zero-terminated
    this->write_byte_in_file<TByte>(l_JIFF_ID, sizeof(l_JIFF_ID) / sizeof(l_JIFF_ID[0]));

    /* set JFIF version */
    TByte l_JIFF_Version[] = {1, 1}; // JFIF version 1.1. Major and Minor version
    this->write_byte_in_file<TByte>(l_JIFF_Version, sizeof(l_JIFF_Version) / sizeof(l_JIFF_Version[0]));

    /* Set density units. Used for resolution */
    TByte l_density_resolution = 0; // no density units specified
    this->write_byte_in_file<TByte>(&l_density_resolution);

    /* Set the horiztonal and vertical density */
    size_t l_horizontal_pixel_density = 1;
    size_t l_vertical_pixel_density = 1;
    /* Density: 1 pixel "per pixel"*/
    write_density_byte_length_in_file(l_horizontal_pixel_density); //setting horizontal density
    write_density_byte_length_in_file(l_vertical_pixel_density);   //setting vertical density

    /* Set thumbnail. No thumbnail. size 0x0 */
    size_t l_X_thumbnail_pixels = 0, l_Y_thumbnail_pixels = 0;
    this->write_byte_in_file<size_t>(&l_X_thumbnail_pixels);
    this->write_byte_in_file<size_t>(&l_Y_thumbnail_pixels);
}

void JPEGCodec::set_JIFF_COM_header()
{
}

void JPEGCodec::calculate_quant_tables(const int &a_quality, float *a_YTable, float *a_UVTable, size_t a_qt_table_size)
{
    /* specifying that the quality level is to be between 1 and 100 */
    int l_quality = std::clamp(a_quality, JFIFConstants::MIN_QUAL, JFIFConstants::MAX_QUAL);

    /* convert to an internal JPEG quality factor, formula taken from libjpeg */
    l_quality = (l_quality < JFIFConstants::MID_QUAL) ? (5000 / l_quality)
                                                      : 200 - l_quality * 2;

    for (auto i = 0; i < a_qt_table_size; i++)
    {
        int luminance = (QuantizationTable::YQT[i] * l_quality + 50) / 100;
        int chrominance = (QuantizationTable::UVQT[i] * l_quality + 50) / 100;

        // clamp to 1..255
        a_YTable[i] = std::clamp(luminance, JFIFConstants::CHAR_MIN, JFIFConstants::CHAR_MAX);
        a_UVTable[i] = std::clamp(chrominance, JFIFConstants::CHAR_MIN, JFIFConstants::CHAR_MAX);
        // a_YTable[hard_coded_zig_zag[i]] = std::clamp(luminance, CHAR_MIN, CHAR_MAX);
        // a_UVTable[hard_coded_zig_zag[i]] = std::clamp(chrominance, CHAR_MIN, CHAR_MAX);
    }
}

/* works */
void JPEGCodec::set_JIFF_DQT_header(const int &a_quality, const int &a_channels, float *a_YTable, float *a_UVTable, size_t a_qt_table_size)
{
    bool l_is_color_img = (a_channels == 3);

    /* write the quantization tables to the jpeg file */
    int l_num_of_tables = (l_is_color_img) ? 2 : 1;
    size_t l_QT_tbl_byte_len = 2 + l_num_of_tables * (1 + a_qt_table_size); /* 2 is for the length of the current field. 65 bytes per table. 65 and not 64 as there is an ID byte the precedes the table */

    /*  Each marker is immediately preceded by an all 1 byte (0xff). */
    TByte l_DQT[] = {JFIFMarkers::START_SECTION, JFIFMarkers::DQT_MARKER};
    this->write_byte_in_file<TByte>(l_DQT, sizeof(l_DQT) / sizeof(l_DQT[0]));

    /* encode length */
    this->write_marker_byte_length_in_file(l_QT_tbl_byte_len);

    /* write lumninace quantization table */
    this->write_byte_in_file<const int>(&JFIFConstants::USE_Y_QT);
    this->write_byte_in_file<float>(a_YTable, a_qt_table_size);

    /* write the second quant table only if it a colour image */
    if (l_is_color_img)
    {
        this->write_byte_in_file<const int>(&JFIFConstants::USE_UV_QT);
        this->write_byte_in_file<float>(a_UVTable, a_qt_table_size);
    }
}

/* works */
void JPEGCodec::set_JIFF_SOF_header(int a_width, int a_height, int a_channels, bool downsample)
{
    // const TByte SOF_MARKER = 0xd8;

    TByte l_SOF[] = {JFIFMarkers::START_SECTION, JFIFMarkers::SOF_MARKER};
    this->write_byte_in_file<TByte>(l_SOF, sizeof(l_SOF) / sizeof(l_SOF[0]));

    /* add length field */
    /* length: 6 bytes general info + 3 * per channel + 2 bytes for this length field */
    int l_field_len = 2;
    size_t l_SOF_length = l_field_len + 6 + (3 * a_channels);
    this->write_marker_byte_length_in_file(l_SOF_length);

    size_t l_data_precision = 0x08; //Data precision - bits per sample. Usually 8 bits
    this->write_byte_in_file<size_t>(&l_data_precision);

    this->write_marker_byte_length_in_file(a_height);
    this->write_marker_byte_length_in_file(a_width);

    /* sampling and quantization tables for each component */
    this->write_byte_in_file<int>(&a_channels); // 1 component (grayscale, Y only) or 3 components (Y,Cb,Cr)
    for (int channel_id = 1; channel_id <= a_channels; channel_id++)
    {
        //                                                                // bitmasks for sampling: highest 4 bits: horizontal, lowest 4 bits: vertical
        this->write_byte_in_file<int>(&channel_id);                          // channel ID (Y=1, Cb=2, Cr=3)
        int l_sampling_rate = (channel_id == 1 && downsample) ? 0x22 : 0x11; // 0x11 is default YCbCr 4:4:4 and 0x22 stands for YCbCr 4:2:0
        this->write_byte_in_file<int>(&l_sampling_rate);
        int l_quant_table_for_channel = (channel_id == 1) ? JFIFConstants::USE_Y_QT : JFIFConstants::USE_UV_QT; // use quantization table 0 for Y, table 1 for Cb and Cr
        this->write_byte_in_file<int>(&l_quant_table_for_channel);
    }
}

/* looks correct */
void JPEGCodec::set_JIFF_DHT_header(const int &a_channels)
{
    bool l_is_color_img = (a_channels == 3);
    int l_HT_YDC_length = 1 + 16 + 12;  // 12 for YDC_categories; 16 for YDC_code_len_amounts
    int l_HT_YAC_length = 1 + 16 + 162; // 16 for std_ac_luminance_nrcodes; 162 for
    int l_HT_UVDC_length = 1 + 16 + 12; // 12 for std_dc_chrominance_values; 16 for std_dc_chrominance_nrcodes
    int l_HT_UVAC_length = 1 + 16 + 162;

    /* 2 bytes for the length of field, store chrominance only if needed*/
    int l_DHT_field_len = (l_is_color_img) ? (2 + l_HT_YDC_length + l_HT_YAC_length + l_HT_UVDC_length + l_HT_UVAC_length)
                                           : (2 + l_HT_YDC_length + l_HT_YAC_length);

    /*  Each marker is immediately preceded by an all 1 byte (0xff). */
    TByte l_DHT[] = {JFIFMarkers::START_SECTION, JFIFMarkers::DHT_MARKER};
    this->write_byte_in_file<TByte>(l_DHT, sizeof(l_DHT) / sizeof(l_DHT[0]));

    /* encode length */
    this->write_marker_byte_length_in_file(l_DHT_field_len);

    // store luminance's DC+AC Huffman table definitions

    this->write_byte_in_file<const TByte>(&JFIFMarkers::luminance_DC_HT_ID);
    this->write_byte_in_file<const unsigned char>(HuffmanTable::YDC_code_len_amounts, 16);
    this->write_byte_in_file<const unsigned char>(HuffmanTable::YDC_categories, 12);

    this->write_byte_in_file<const TByte>(&JFIFMarkers::luminance_AC_HT_ID);
    this->write_byte_in_file<const unsigned char>(HuffmanTable::std_ac_luminance_nrcodes, 16);
    this->write_byte_in_file<const unsigned char>(HuffmanTable::std_ac_luminance_values, 162);

    // chrominance is only relevant for color images
    if (l_is_color_img)
    {
        // store luminance's DC+AC Huffman table definitions
        this->write_byte_in_file<const TByte>(&JFIFMarkers::chrominance_DC_HT_ID);
        this->write_byte_in_file<const unsigned char>(HuffmanTable::std_dc_chrominance_nrcodes, 16);
        this->write_byte_in_file<const unsigned char>(HuffmanTable::std_dc_chrominance_values, 12);

        this->write_byte_in_file<const TByte>(&JFIFMarkers::chrominance_AC_HT_ID);
        this->write_byte_in_file<const unsigned char>(HuffmanTable::std_ac_chrominance_nrcodes, 16);
        this->write_byte_in_file<const unsigned char>(HuffmanTable::std_ac_chrominance_values, 162);
    }
}

/* looks right */
void JPEGCodec::set_JIFF_SOS_header(int a_channels)
{
    /*  Each marker is immediately preceded by an all 1 byte (0xff). */
    TByte l_SOS[] = {JFIFMarkers::START_SECTION, JFIFMarkers::SOS_MARKER};
    this->write_byte_in_file<TByte>(l_SOS, sizeof(l_SOS) / sizeof(l_SOS[0]));

    /* encode length */
    int l_field_len = 2;
    /* length: 6 bytes general info + 3 * per channel + 2 bytes for this length field */
    int l_SOS_length = 2 + 1 + 2 * a_channels + 3; // 2 bytes for the length field, 1 byte for number of components,
                                                   // then 2 bytes for each component and 3 bytes for spectral selection
    this->write_marker_byte_length_in_file(l_SOS_length);

    // assign Huffman tables to each component
    this->write_byte_in_file<int>(&a_channels);
    for (int channel_id = 1; channel_id <= a_channels; channel_id++)
    {
        this->write_byte_in_file<int>(&channel_id);
        int l_dc_tbl_chosen = (channel_id == 1) ? 0x00 : 0x10; // highest 4 bits: DC Huffman table, lowest 4 bits: AC Huffman table
        int l_ac_tbl_chosen = (channel_id == 1) ? 0x00 : 0x01;
        int l_ac_dc_tbls_chosen = l_dc_tbl_chosen | l_ac_tbl_chosen;
        this->write_byte_in_file<int>(&l_ac_dc_tbls_chosen); // Y: tables 0 for DC and AC; Cb + Cr: tables 1 for DC and AC
    }

    /* constant values for our baseline JPEGs (which have a single sequential scan) */
    const uint8_t l_spectral[3] = {0, 63, 0}; // spectral selection: must be from 0 to 63; successive approximation must be 0
    this->write_byte_in_file<const uint8_t>(l_spectral, sizeof(l_spectral) / sizeof(l_spectral[0]));
    /* after this is the start of the actual encoding */
}
