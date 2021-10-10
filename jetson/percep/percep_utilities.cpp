#include <cmath>

// Coverts the input degree (and optional minute) to radians.
double degreeToRadian( const double degree, const double minute )
{
    return ( PI / 180 ) * ( degree + minute / 60 );
} // degreeToRadian

// Converts the input radians to degrees.
double radianToDegree( const double radian )
{
    return radian * 180 / PI;
}

// Caclulates the non-euclidean distance between the current odometry and the
// destination odometry.
double estimateNoneuclid( const Odometry& current, const Odometry& dest )
{
    double currentLat = degreeToRadian( current.latitude_deg, current.latitude_min );
    double currentLon = degreeToRadian( current.longitude_deg, current.longitude_min );
    double destLat = degreeToRadian( dest.latitude_deg, dest.latitude_min );
    double destLon = degreeToRadian( dest.longitude_deg, dest.longitude_min );

    double diffLat = ( destLat - currentLat );
    double diffLon = ( destLon - currentLon ) * cos( ( currentLat + destLat ) / 2 );
    return sqrt( diffLat * diffLat + diffLon * diffLon ) * EARTH_RADIUS;
}