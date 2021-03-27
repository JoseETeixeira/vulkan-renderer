#pragma once

namespace inexor::vulkan_renderer::wrapper {

/// @brief A small helper function that return vulkan create infos with sType already set
/// @code{.cpp}
/// auto render_pass_ci = make_info<VkRenderPassCreateInfo>();
/// @endcode
/// @note Also zeros the returned struct
template <typename T>
[[nodiscard]] T make_info();

} // namespace inexor::vulkan_renderer::wrapper
