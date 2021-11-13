#ifndef _PROJEKT_LOGS_
#define _PROJEKT_LOGS_
#include <spdlog/formatter.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <Eigen/dense>
#include <Eigen/geometry>

using namespace Eigen;


template<typename Ostream>
Ostream & operator<<(Ostream & os, const Eigen::Vector2f & val)
{
	return os << "(" << val.x() << ", " << val.y() << ")";
}

template <typename OutputIt, typename CharT>
OutputIt copy_str(OutputIt it, const CharT s[])
{
    const CharT* c = s;
    while(*c)
    {
        *it++ = *c;
        c++;
    }
    return it;
}

template <typename Scalar_, int Rows_, int Cols_>
struct fmt::formatter<Eigen::Matrix<Scalar_, Rows_, Cols_>>: fmt::formatter<float>
{
    template<typename FormatContext>
    auto format(const Eigen::Matrix<Scalar_, Rows_, Cols_>& v,
        FormatContext& ctx)->decltype(ctx.out())
    {
        copy_str(ctx.out(), "(");
        for(int i = 0; i<Rows_-1; i++)
        {
            for(int j = 0; j<Cols_-1; j++)
            {
                fmt::formatter<Scalar_>::format(v(i, j), ctx);
                *ctx.out() = ',';
                *ctx.out() = ' ';
            }
            fmt::formatter<Scalar_>::format(v(i, Cols_ - 1), ctx);
            copy_str(ctx.out(), "; ");
        }
        fmt::formatter<Scalar_>::format(v(Rows_ - 1, Cols_ -1), ctx);
        copy_str(ctx.out(), ")");
        return ctx.out();
    }
};

extern std::shared_ptr<spdlog::logger> logger;
template <typename FormatString, typename... Args> 
inline void dbgmsg(const FormatString& fmt, Args&&...args)
{
#ifdef _DEBUG
	spdlog::get("basic_logger")->info(fmt, args...);
#endif // _DEBUG

}

#endif // !_PROJEKT_LOGS_