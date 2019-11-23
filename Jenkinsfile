pipeline {
    agent any

    environment {
        // env GIT_BRANCH looks like `origin/branch` Subsitution performes: 1. `originbranch` 2. `branch`
        BRANCH =    """${sh(    returnStdout: true,
                                script: 'echo $GIT_BRANCH | sed s/[/]//g | sed s/origin//g'
                        )} 
                    """.trim()
        // TBD: Confirm cpu/memory flag actually works as expected
        DOCKER_BUILD_FLAGS = "--cpu-shares=100 --memory=512m --build-arg CPP_MAKE_FLAGS=\'--jobs 1 --max-load 1.7\'" 
    }
    
    // Kill build if taking too long
    options {
        timeout(time:3 , unit: 'HOURS')
    }

    stages {
        stage('Build Video Record') {
            steps {
                sh  '''#!/bin/bash -xe
                    echo 'Building Video Record..';
                    pushd video_record > /dev/null;
                    make build-docker   GIT_COMMIT=$GIT_COMMIT \
                                        GIT_REPO_URL=$GIT_URL \
                                        GIT_BRANCH=$BRANCH \
                                        IMAGE_TAG=$BRANCH \
                                        DOCKER_BUILD_FLAGS="$DOCKER_BUILD_FLAGS";
                    popd > /dev/null;
                '''
            }
        }
        stage('Push Video Record') {
            steps {
                sh  '''#!/bin/bash -xe
                    echo 'Pushing Video Record..';
                    pushd video_record > /dev/null;
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
                    make build-docker   GIT_COMMIT=$GIT_COMMIT \
                                        GIT_REPO_URL=$GIT_URL \
                                        GIT_BRANCH=$BRANCH \
                                        IMAGE_TAG=$BRANCH \
                                        DOCKER_BUILD_FLAGS="$DOCKER_BUILD_FLAGS";
                    popd > /dev/null;
                '''
            }
        }
        stage('Push View Rosbag') {
            steps {
                sh  '''#!/bin/bash -xe
                    echo 'Pushing View Rosbag..';
                    pushd view_rosbag > /dev/null;
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
        always{
            sh  '''#!/bin/bash +ex
                    docker rmi techye/video_record:$BRANCH
                    docker rmi techye/view_rosbag:$BRANCH
                    # docker image prune -a -f --filter "label!=techye/ci"
                    docker system prune -f
                '''
        }
        success{
            withCredentials([string(credentialsId: 'statusToken', variable: 'TOKEN')]) {
                sh  '''#!/bin/bash
                        echo GIT COMMIT: $GIT_COMMIT
                        echo BUILD NUMBER: $BUILD_NUMBER
                        curl "https://api.GitHub.com/repos/techye/ros-example/statuses/$GIT_COMMIT?access_token=$TOKEN" \
                            -H "Content-Type: application/json" \
                            -X POST \
                            -d '{"state": "success", "context": "Jenkins", "description": "CI/CD Pipeline", "target_url": "http://52.31.241.174:8080/job/ros-example/$BUILD_NUMBER/console"}'
                        '''
            }
        }
        failure{
            withCredentials([string(credentialsId: 'statusToken', variable: 'TOKEN')]) {
                sh  '''#!/bin/bash
                        echo GIT COMMIT: $GIT_COMMIT
                        echo BUILD NUMBER: $BUILD_NUMBER
                        curl "https://api.GitHub.com/repos/techye/ros-example/statuses/$GIT_COMMIT?access_token=TOKEN" \
                            -H "Content-Type: application/json" \
                            -X POST \
                            -d '{"state": "failure", "context": "Jenkins", "description": "CI/CD Pipeline", "target_url": "http://52.31.241.174:8080/job/ros-example/$BUILD_NUMBER/console"}'
                    '''
            }
        }
    }
}
