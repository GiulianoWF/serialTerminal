#include <iostream>

#include <boost/iostreams/stream.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>

using boost::asio::serial_port_base;
int main()
{
    boost::asio::io_service ioService;

    auto port = boost::asio::serial_port(ioService);

    boost::system::error_code ec;
    port.open("/dev/ttyACM0", ec);

    auto opt_baud = boost::asio::serial_port_base::baud_rate(9600);
    auto opt_parity = serial_port_base::parity(serial_port_base::parity::none);
    auto opt_csize = serial_port_base::character_size(8);
    auto opt_flow = serial_port_base::flow_control(serial_port_base::flow_control::none);
    auto opt_stop = serial_port_base::stop_bits(serial_port_base::stop_bits::one);

    port.set_option(opt_baud);
    port.set_option(opt_parity);
    port.set_option(opt_csize);
    port.set_option(opt_flow);
    port.set_option(opt_stop);    
    
    unsigned char readBuffer[1024];

    std::function<void(const boost::system::error_code&, size_t)> readSerial;

    readSerial = [&port, &readSerial, &readBuffer](const boost::system::error_code& ec, size_t bytes_transferred)
    {
        for(int i = 0; i < bytes_transferred; ++i)
        {
            std::cout << readBuffer[i];
        }
        port.async_read_some(boost::asio::buffer(readBuffer, 512), readSerial);
    };

    port.async_read_some(boost::asio::buffer(readBuffer, 512), readSerial);

    auto readThread = std::thread([&ioService](){
        ioService.run();
    });

    readThread.detach();

    std::string userCommand = "";
    bool exit = false;
    
    while(false == exit)
    {
        std::cin >> userCommand;

        if(userCommand.length() > 1)
        {
            if ('q' == userCommand.at(0))
            {
                exit = true;
            }
        }

        port.write_some(boost::asio::buffer(userCommand));
    }
}
