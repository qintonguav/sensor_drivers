#include <unistd.h>
#include "Timer.h"

//=============================================================================
//======================= CTime implementation ================================
//=============================================================================

//-----------------------------------------------------------------------------
CTime::CTime()
//-----------------------------------------------------------------------------
{
    start( );
}

//-----------------------------------------------------------------------------
long CTime::tvToMSec( const struct timeval tv )
//-----------------------------------------------------------------------------
{
    return( tv.tv_sec * 1000 + tv.tv_usec / 1000 );
}

//-----------------------------------------------------------------------------
long CTime::tvToUSec( const struct timeval tv )
//-----------------------------------------------------------------------------
{
    return( tv.tv_sec * 1000000 + tv.tv_usec );
}

//-----------------------------------------------------------------------------
double CTime::elapsed( void )
//-----------------------------------------------------------------------------
{
    gettimeofday( &tvEnd, NULL ) ;
    static struct timeval tvDiff;
    tvDiff.tv_sec  = tvEnd.tv_sec  - tvStart.tv_sec ;
    tvDiff.tv_usec = tvEnd.tv_usec - tvStart.tv_usec ;

    if( tvDiff.tv_usec < 0 ) // handle 1 sec borrow
    {
        tvDiff.tv_usec += 1000000 ; // usec / sec
        tvDiff.tv_sec -= 1 ;
    }
    return static_cast<double>( tvToMSec( tvDiff ) / 1000.0 ); // in sec
}

//-----------------------------------------------------------------------------
double CTime::restart( void )
//-----------------------------------------------------------------------------
{
    double result = elapsed( );
    tvStart = tvEnd;
    return result;
}

//-----------------------------------------------------------------------------
void CTime::start( void )
//-----------------------------------------------------------------------------
{
    gettimeofday( &tvStart, NULL );
}

