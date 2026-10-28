from setuptools import setup, find_packages

package_name = "robot_navigator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="your@email.com",
    description="LLM-powered LiDAR navigation robot",
    license="MIT local",
    entry_points={
        "console_scripts": [
            "robot_agent = agent.agent:main",
            "robot_dashboard = dashboard.server:main",
        ],
    },
)
