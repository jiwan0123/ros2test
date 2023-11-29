from setuptools import find_packages, setup  # 패키지를 찾아주는 함수, 패키지를 설치하는 함수

package_name = 'fibonacci_action_py'  # 패키지 이름

setup(  # 패키지를 설치하는 함수
    name=package_name,  # 패키지 이름
    version='0.0.0',  # 패키지 버전
    packages=find_packages(exclude=['test']),  # 패키지를 찾아주는 함수
    data_files=[  # 패키지에 포함되는 데이터 파일
        ('share/ament_index/resource_index/packages',  # 패키지의 데이터 파일
            ['resource/' + package_name]),  # 패키지의 데이터 파일
        ('share/' + package_name, ['package.xml']),  # 패키지의 데이터 파일
    ],
    install_requires=['setuptools'],  # 패키지 설치에 필요한 패키지
    zip_safe=True,  # 패키지를 압축하여 설치
    maintainer='sis',  # 패키지 유지자
    maintainer_email='sis@todo.todo',  # 패키지 유지자의 이메일
    description='ros2 python fibonacci action',  # 패키지 설명
    license='Apache License 2.0',  # 패키지 라이센스
    tests_require=['pytest'],  # 패키지 테스트에 필요한 패키지
    entry_points={  # 패키지의 실행 파일
        'console_scripts': [  # 패키지의 실행 파일
            'server = fibonacci_action_py.server:main',  # 패키지의 실행 파일
            'client = fibonacci_action_py.client:main',  # 패키지의 실행 파일
        ],
    },
)
