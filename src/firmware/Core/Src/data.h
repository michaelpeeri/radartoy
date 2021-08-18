#include <utility>

/* TODO FIX THIS LATER!!!
template<typename It1, typename It2, int D>
void convert(It1 begin1, It1 end1, It2 begin2, It2 end2)
{
    for(auto it1=begin1, it2=begin2; it1!=end1; ++it1, ++it2 )
        *it2 = static_cast<decltype(std::declval(*it2))>(*it1) - D;
}
*/

template<typename T1, typename T2, int D>
void convert(const T1* begin1, const T1* end1, T2* begin2, T2* end2)
{
    const T1* it1=begin1;
    T2*       it2=begin2;
    for(; it1!=end1; ++it1, ++it2 )
        *it2 = static_cast<T2>(*it1) - D;
}