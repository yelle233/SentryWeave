from setuptools import setup

setup(
    name="sentry_core", version="0.1.0", packages=["sentry_core"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/sentry_core"]),
                ("share/sentry_core", ["package.xml"])],
    install_requires=["setuptools", "numpy", "PyYAML"], zip_safe=True,
    maintainer="yelle233", maintainer_email="189729296+yelle233@users.noreply.github.com",
    description="Cooperative sentry decision core", license="Apache-2.0",
)
