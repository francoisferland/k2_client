// Based on k2_client/rgb structure, but outputs an H.264 stream on port 9100.

#include <boost/asio.hpp>

const size_t IMAGE_WIDTH  = 1920;
const size_t IMAGE_HEIGHT = 1080;
const size_t IMAGE_SIZE   = IMAGE_WIDTH * IMAGE_HEIGHT * 3;      // 24-bit RGB image
const size_t FRAME_SIZE   = IMAGE_SIZE  + sizeof(unsigned long); // image + timestamp 

unsigned char frame_buffer_[FRAME_SIZE];

int main(int argc, char *argv[])
{
    using namespace boost::asio::ip;

    // TODO: Proper parameters.
    std::string server_host("127.0.0.1");
    std::string server_port("9000");

    // Create a Boost ASIO service to handle server connection.
    boost::asio::io_service io_service;

    // Get a list of endpoints corresponding to the server hostname and port.
    tcp::resolver resolver(io_service);
    tcp::resolver::iterator endpoint_iterator = resolver.resolve({server_host, server_port});

    // Try each endpoint until we successfully establish a connection.
    tcp::socket socket(io_service);
    try
    {
        boost::asio::connect(socket, endpoint_iterator);
    }
    catch (boost::system::system_error const& e)
    {
        //ROS_FATAL("Failed to connect to k2 server '%s:%s': %s",
        //          server_host.c_str(), server_port.c_str(), e.what());
        return -1;
    }

    while(true)
    {
        // Read the next image from the server.
        boost::asio::read(socket, boost::asio::buffer(frame_buffer_, FRAME_SIZE));

        // Extract the timestamp (packed at end of image).
        unsigned long timestamp = *reinterpret_cast<unsigned long *>(&frame_buffer_[IMAGE_SIZE]);

        // TODO: ...
    }

    return 0;
}
