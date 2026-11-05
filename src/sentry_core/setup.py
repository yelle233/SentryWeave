from setuptools import setup

setup(
    name="sentry_core", version="0.1.0", packages=["sentry_core"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/sentry_core"]),
                ("share/sentry_core", ["package.xml"])],
    install_requires=["setuptools", "numpy", "PyYAML"], zip_safe=True,
    maintainer="yelle233 (未来战队)", maintainer_email="1376982098@qq.com",
    description="Cooperative sentry decision core", license="Apache-2.0",
)
