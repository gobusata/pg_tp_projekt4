#ifndef _PROJEKT_LOGS_
#define _PROJEKT_LOGS_
#include <spdlog/formatter.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <Eigen/dense>
#include <Eigen/geometry>

using namespace Eigen;

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
struct fmt::formatter<Eigen::Matrix<Scalar_, Rows_, Cols_>>: fmt::formatter<Scalar_>
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

template <typename Scalar_>
struct fmt::formatter<Eigen::Matrix<Scalar_, Eigen::Dynamic, Eigen::Dynamic>> : fmt::formatter<float>
{
    template<typename FormatContext>
    auto format(const Eigen::Matrix<Scalar_, Eigen::Dynamic, Eigen::Dynamic>& v,
        FormatContext& ctx)->decltype(ctx.out())
    {
        Eigen::Index r_ = v.rows(), c_ = v.cols();
        copy_str(ctx.out(), "(");
        for (int i = 0; i < r_; i++)
        {
            for (int j = 0; j < c_ - 1; j++)
            {
                fmt::formatter<Scalar_>::format(v(i, j), ctx);
                *ctx.out() = ',';
                *ctx.out() = ' ';
            }
            fmt::formatter<Scalar_>::format(v(i, c_ - 1), ctx);
            if (i == r_ - 1)
                copy_str(ctx.out(), ") ");
            else
                copy_str(ctx.out(), "; ");
        }
        return ctx.out();
    }
};

extern std::shared_ptr<spdlog::logger> logger;

inline void dbginit()
{
#ifdef _DEBUG 
    spdlog::get("basic_logger")->set_pattern("%v");
#endif // _DEBUG
}

template <typename FormatString, typename... Args> 
inline void dbgmsg(const FormatString& fmt, Args&&...args)
{
#ifdef _DEBUG
	spdlog::get("basic_logger")->info(fmt, args...);
#endif // _DEBUG

}


#endif // !_PROJEKT_LOGS_