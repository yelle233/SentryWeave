from setuptools import setup

setup(
    name="sentry_ros", version="0.1.0", packages=["sentry_ros"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/sentry_ros"]),
                ("share/sentry_ros", ["package.xml"])],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="yelle233 (未来战队)", maintainer_email="1376982098@qq.com",
    description="Jazzy cooperative sentry runtime", license="Apache-2.0",
    entry_points={"console_scripts": [
        "world_model = sentry_ros.world_node:main",
        "decision = sentry_ros.decision_node:main",
        "executor = sentry_ros.executor_node:main",
        "velocity_guard = sentry_ros.guard_node:main",
    ]},
)
