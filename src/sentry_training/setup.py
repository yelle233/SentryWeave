from setuptools import find_packages, setup

setup(
    name="sentry_training", version="0.1.0", packages=find_packages(include=["sentry_training*"]),
    data_files=[("share/ament_index/resource_index/packages", ["resource/sentry_training"]),
                ("share/sentry_training", ["package.xml"])],
    install_requires=["setuptools", "numpy"], zip_safe=True,
    maintainer="yelle233", maintainer_email="189729296+yelle233@users.noreply.github.com",
    description="Sentry training and replay", license="Apache-2.0",
    entry_points={"console_scripts": [
        "sentry_train = sentry_training.cli:main",
        "sentry_rl = sentry_training.offline.cli:main",
    ]},
)
