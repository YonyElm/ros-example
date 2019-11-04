pipeline {
    agent any

    environment {
        BRANCH =    """${sh(   returnStdout: true,
                            script: 'echo $GIT_BRANCH')}
                    """.trim()
    }
    
    stages {
        stage('Build Video Record') {
            steps {
                sh  '''#!/bin/bash -xe
                    echo 'Building Video Record..';
                    pushd video_record > /dev/null;
                    make build-docker IMAGE_TAG=$BRANCH;
                    make push-docker IMAGE_TAG=$BRANCH;
                    popd > /dev/null;
                '''
            }
        }
        stage('Build View Rosbag') {
            steps {
                sh  '''#!/bin/bash -xe
                    echo 'Building View Rosbag..';
                    pushd view_rosbag > /dev/null;
                    make build-docker IMAGE_TAG=$BRANCH;
                    make push-docker IMAGE_TAG=$BRANCH;
                    popd > /dev/null;
                '''
            }
        }
        stage('Test') {
            steps {
                echo 'Testing..'
            }
        }
        stage('Deploy') {
            steps {
                echo 'Deploying....'
            }
        }
    }

    // Post build - will be executed after build is done
    post {
        success{
            sh  '''#!/bin/bash
                    echo GIT COMMIT: $GIT_COMMIT
                    echo BUILD NUMBER: $BUILD_NUMBER
                    curl "https://api.GitHub.com/repos/techye/ros-example/statuses/$GIT_COMMIT?access_token=4e2984b191e61afa65b6e4b044cf321846e164f8" \
                        -H "Content-Type: application/json" \
                        -X POST \
                        -d '{"state": "success", "context": "Jenkins", "description": "CI/CD Pipeline", "target_url": "http://52.31.241.174:8080/job/ros-example/$BUILD_NUMBER/console"}'
                '''
        }
        failure{
            sh  '''#!/bin/bash
                    echo GIT COMMIT: $GIT_COMMIT
                    echo BUILD NUMBER: $BUILD_NUMBER
                    curl "https://api.GitHub.com/repos/techye/ros-example/statuses/$GIT_COMMIT?access_token=4e2984b191e61afa65b6e4b044cf321846e164f8" \
                        -H "Content-Type: application/json" \
                        -X POST \
                        -d '{"state": "failure", "context": "Jenkins", "description": "CI/CD Pipeline", "target_url": "http://52.31.241.174:8080/job/ros-example/$BUILD_NUMBER/console"}'
                '''
        }
    }
}
