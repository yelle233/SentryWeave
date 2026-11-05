from setuptools import setup

setup(
    name="sentry_ros", version="0.1.0", packages=["sentry_ros"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/sentry_ros"]),
                ("share/sentry_ros", ["package.xml"])],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="yelle233", maintainer_email="189729296+yelle233@users.noreply.github.com",
    description="Jazzy cooperative sentry runtime", license="Apache-2.0",
    entry_points={"console_scripts": [
        "world_model = sentry_ros.world_node:main",
        "decision = sentry_ros.decision_node:main",
        "executor = sentry_ros.executor_node:main",
        "velocity_guard = sentry_ros.guard_node:main",
        "scenario = sentry_ros.scenario_node:main",
        "sim_localization = sentry_ros.localization_node:main",
    ]},
)
