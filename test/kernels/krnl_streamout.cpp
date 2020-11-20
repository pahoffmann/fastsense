#include <hls_stream.h>
#include <ap_axi_sdata.h>

constexpr int NUM = 256;

extern "C"
{
    void krnl_streamout(int* in, hls::stream<ap_axiu<32, 0, 0, 0>>& out)
    {
        for(int i = 0; i < NUM; i++)
        {
            ap_axiu<32, 0, 0, 0> data;
            data.data = in[i];
            data.last = i == (NUM-1);
            out << data;
        }
    }
};