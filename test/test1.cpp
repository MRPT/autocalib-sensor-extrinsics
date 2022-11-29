/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#define BOOST_TEST_MODULE my_test1
#include <boost/test/unit_test.hpp>

int add( int i, int j ) { return i+j; }

BOOST_AUTO_TEST_CASE( test_case_boost )
{
    // seven ways to detect and report the same error:
	BOOST_CHECK( add( 2,2 ) == 4 );        // #1 continues on error

	BOOST_REQUIRE( add( 2,2 ) == 4 );      // #2 throws on error

	if( add( 2,2 ) != 4 )
		BOOST_ERROR( "Ouch..." );          // #3 continues on error

	if( add( 2,2 ) != 4 )
		BOOST_FAIL( "Ouch..." );           // #4 throws on error

	if( add( 2,2 ) != 4 ) throw "Ouch..."; // #5 throws on error

	BOOST_CHECK_MESSAGE( add( 2,2 ) == 4,  // #6 continues on error
			"add(..) result: " << add( 2,2 ) );

	BOOST_CHECK_EQUAL( add( 2,2 ), 4 );	  // #7 continues on error
}
