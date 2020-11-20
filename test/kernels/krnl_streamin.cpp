#include <hls_stream.h>
#include <ap_axi_sdata.h>

constexpr int NUM = 256;

extern "C"
{
    void krnl_streamin(int* out, hls::stream<ap_axiu<32, 0, 0, 0>>& in)
    {
        for(int i = 0; i < NUM; i++)
        {
            ap_axiu<32, 0, 0, 0> data;
            in >> data;
            out[i] = data.data;
        }
    }
};