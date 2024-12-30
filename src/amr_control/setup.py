from setuptools import setup

package_name = 'fullfilment'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[package_name + '.amr_control.robot_node'],  # 실행 파일
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='AMR control with robot node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_node = fullfilment.amr_control.robot_node:main',  # 엔트리 포인트
        ],
    },
)
