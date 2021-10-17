#include "ProjektLogs.h"

//using namespace Eigen;
//
//template <>
//struct fmt::formatter<Vector2f>
//{
//	char presentation = 'f';
//	constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin())
//	{
//		auto it = ctx.begin(), end = ctx.end();
//		return end;
//	}
//
//	template <typename FormatContext>
//	auto format(const Vector2f& p, FormatContext& ctx) -> decltype(ctx.out()) {
//		return format_to(
//			ctx.out(),
//			"({:.2f}, {:.2f})",
//			p[0], p[1]);
//	}
//};