
#include "../geometry/spline.h"
#include "debug.h"

template<typename T>
T Spline<T>::cubic_unit_spline(float time, const T& position0, const T& position1,
                               const T& tangent0, const T& tangent1) {

    // TODO (Animation): Task 1a
    // Given time in [0,1] compute the cubic spline coefficients and use them to compute
    // the interpolated value at time 'time' based on the positions & tangents

    // Note that Spline is parameterized on type T, which allows us to create splines over
    // any type that supports the * and + operators.
    float square = time * time;
    float cube = square * time;
    float h00 = 2.0f * cube - 3.0f * square + 1.0f;
    float h10 = cube - 2.0f * square + time;
    float h01 = -2.0f * cube + 3.0f * square;
    float h11 = cube - square;




    return h00 * position0 + h10 * tangent0 + h01 * position1 + h11 * tangent1;
}

template<typename T> T Spline<T>::at(float time) const {

    // TODO (Animation): Task 1b

    // Given a time, find the nearest positions & tangent values
    // defined by the control point map.

    // Transform them for use with cubic_unit_spline

    // Be wary of edge cases! What if time is before the first knot,
    // before the second knot, etc...
    if (!any()){
        return T();
    }
    if (control_points.size() == 1){
        return control_points.begin()->second;
    }

    auto timeIt = control_points.upper_bound(time);
    if (timeIt == control_points.end()){
        timeIt--;
        // printf("Too big! out of bound!\n");
        return timeIt->second;
    }
    else if (timeIt == control_points.begin()){
        // printf("Too small! out of bound!\n");
        return timeIt->second;
    }

    float k2 = timeIt->first;
    const T& p2 = timeIt->second;

    auto time1It = timeIt;
    time1It--;
    float k1 = time1It->first;
    const T& p1 = time1It->second;

    float k0;
    T p0 = p1 - (p2 - p1);
    if (control_points.begin() == time1It){
        k0 = k1 - (k2 - k1);
        // printf("k0: %f\n", k0);
        // p0 = p1 - (p2 - p1);
    }
    else{
        time1It --;
        k0 = time1It->first;
        p0 = time1It->second;
    }
    float k3;
    T p3 = timeIt->second;
    if (control_points.end() == timeIt){
        
        k3 = k2 + (k2 - k1);
        // printf("k3: %f\n", k3);
        p3 = p2 + (p2 - p1);
    }
    else{
        timeIt++;
        k3 = timeIt->first;
        p3 = timeIt->second;
    }

    T m1 = (p2 - p0) / (k2 - k0);
    T m2 = (p3 - p1) / (k3 - k1);

    float interval = k2 - k1;
    float normalizedTime = (time - k1) / interval;

    
    m1 *= interval;
    m2 *= interval;
    


    return cubic_unit_spline(normalizedTime, p1, p2, m1, m2);
}
