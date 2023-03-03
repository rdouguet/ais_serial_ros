/*
 * Copyright © 2022 , Universite Bretagne Sud, Lab-STICC, Ronan Douguet & Dominique Heller
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the “Software”), to deal in the Software
 * without restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies
 * or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
 * OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef IF_SERIAL_BOOST_HPP
#define IF_SERIAL_BOOST_HPP

/* includes ----------------------------------------------------------------*/
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <serial/serial.h>

#include <marnav/io/default_nmea_reader.hpp>

/* defines -----------------------------------------------------------------*/
/* strutures ---------------------------------------------------------------*/
/* constant ----------------------------------------------------------------*/
/* global variables --------------------------------------------------------*/
/* class -------------------------------------------------------------------*/
class IfSerialBoost : public marnav::io::device
{
public:

  	/*==============================================================*/
  	/* Constructor                                                  */
  	/*==============================================================*/
	IfSerialBoost(const std::string & port, int baud_rate = 115200)
		: io()
		, serial(io, port)
	{
		serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
	}

	/*==============================================================*/
  	/* Pulic methods                                                */
  	/*==============================================================*/

    /**
    @fn virtual void open() override{}
      Open the USB virtual port (define in marnav::io::device)
  	*/
	virtual void open() override{};

    /**
    @fn virtual void open() override{}
      Close the USB virtual port (define in marnav::io::device)
  	*/
	virtual void close() override { serial.close(); }

    /**
    @fn virtual int read(char * buffer, uint32_t size) override
      Read data on the USB virtual port (define in marnav::io::device)
  	*/
	virtual int read(char * buffer, uint32_t size) override
	{
		if ((buffer == nullptr) || (size == 0))
		{
			throw std::invalid_argument{"invalid buffer or size"};
		}

		if (!serial.is_open())
		{
			throw std::runtime_error{"device not open"};
		}

		return boost::asio::read(serial, boost::asio::buffer(buffer, size));
	}

    /**
    @fn virtual int write(const char * buffer, uint32_t size) override
      Write data on the USB virtual port (define in marnav::io::device)
  	*/
	virtual int write(const char * buffer, uint32_t size) override
	{
		if (buffer == nullptr)
		{
			throw std::invalid_argument{"invalid buffer"};
		}

		if (!serial.is_open())
		{
			throw std::runtime_error{"device not open"};
		}

		return boost::asio::write(serial, boost::asio::buffer(buffer, size));
	}

private:

  	/*==============================================================*/
  	/* Private variables                                            */
  	/*==============================================================*/
	boost::asio::io_service io;
	boost::asio::serial_port serial;
};


#endif // IF_SERIAL_BOOST_HPP

