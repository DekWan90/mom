#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

static void help( char* progName )
{
    std::cout   << std::endl
                << "This program shows how to filter images with mask: the write it yourself and the"
                << "filter2d way. " << std::endl
                << "Usage:" << std::endl
                << progName << " [image_path -- default lena.jpg] [G -- grayscale] " << std::endl << std::endl;
}

void Sharpen( const cv::Mat& myImage, cv::Mat& Result );

int main( int argc, char* argv[] )
{
    help( argv[0] );
    const char* filename = argc >=2 ? argv[1] : "lena.jpg";

    cv::Mat src, dst0, dst1;

    if( argc >= 3 && !strcmp( "G", argv[2] ) ) src = imread( cv::samples::findFile( filename ), cv::IMREAD_GRAYSCALE);
    else src = cv::imread( cv::samples::findFile( filename ), cv::IMREAD_COLOR);

    if( src.empty() )
    {
        std::cerr << "Can't open image ["  << filename << "]" << std::endl;
        return EXIT_FAILURE;
    }

    cv::namedWindow( "Input", cv::WINDOW_KEEPRATIO );
    imshow( "Input", src );
    double t = (double) cv::getTickCount();

    Sharpen( src, dst0 );

    t = ( ( double ) cv::getTickCount() - t ) / cv::getTickFrequency();
    std::cout << "Hand written function time passed in seconds: " << t << std::endl;

    cv::namedWindow( "Output 1", cv::WINDOW_KEEPRATIO );
    cv::imshow( "Output 1", dst0 );
    cv::waitKey();

    //![kern]
    cv::Mat kernel = ( cv::Mat_<char>(3,3) <<   0, -1,  0,
                                                -1,  5, -1,
                                                0, -1,  0 );
    //![kern]

    t = ( double ) cv::getTickCount();

    //![filter2D]
    filter2D( src, dst1, src.depth(), kernel );
    //![filter2D]
    t = ( ( double ) cv::getTickCount() - t ) / cv::getTickFrequency();
    std::cout << "Built-in filter2D time passed in seconds:     " << t << std::endl;

    cv::namedWindow( "Output 2", cv::WINDOW_KEEPRATIO );
    cv::imshow( "Output 2", dst1 );

    cv::waitKey();
    return EXIT_SUCCESS;
}
//! [basic_method]
void Sharpen( const cv::Mat& myImage, cv::Mat& Result )
{
    //! [8_bit]
    CV_Assert(myImage.depth() == CV_8U);  // accept only uchar images
    //! [8_bit]

    //! [create_channels]
    const int nChannels = myImage.channels();
    Result.create(myImage.size(),myImage.type());
    //! [create_channels]

    //! [basic_method_loop]
    for(int j = 1 ; j < myImage.rows-1; ++j)
    {
        const uchar* previous = myImage.ptr<uchar>( j - 1 );
        const uchar* current  = myImage.ptr<uchar>( j     );
        const uchar* next     = myImage.ptr<uchar>( j + 1 );

        uchar* output = Result.ptr<uchar>( j );

        for( int i = nChannels; i < nChannels * ( myImage.cols - 1 ); ++i )
        {
            *output++ = cv::saturate_cast<uchar>( 5 * current[i] - current[i - nChannels] - current[i + nChannels] - previous[i] - next[i] );
        }
    }
    //! [basic_method_loop]

    //! [borders]
    Result.row( 0 ).setTo( cv::Scalar(0) );
    Result.row( Result.rows - 1 ).setTo( cv::Scalar( 0 ) );
    Result.col( 0 ).setTo( cv::Scalar( 0 ) );
    Result.col( Result.cols - 1 ).setTo( cv::Scalar( 0 ) );
    //! [borders]
}
//! [basic_method]
