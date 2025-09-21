add_rules("mode.debug", "mode.release")

add_requires("pcl", "eigen","nlohmann_json","thread-pool","csvparser","openmp","boost","nanoflann","cmake")

package("shark")
    set_kind("library")
    set_homepage("https://github.com/Shark-ML/Shark")
    set_description("Shark Machine Learning Library")

    add_urls("https://github.com/Shark-ML/Shark.git")
    add_versions("master", "latest")

    on_install(function (package)
        import("package.tools.cmake").install(package, {
            "-DSHARK_BUILD_EXAMPLES=OFF",
            "-DSHARK_BUILD_TESTS=OFF",
            "-DSHARK_BUILD_DOCUMENTATION=OFF"
        })
    end)
package_end()

target("pointcloud_registration")
    set_kind("binary")
    add_files("src/*.cpp")
    set_languages("c++23")
    add_includedirs("src")
    add_packages("pcl", "eigen","nlohmann_json","thread-pool","csvparser","openmp","nanoflann","shark")
target_end()