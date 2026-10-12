#include "scanner/imagethread.h"
#include "imagefileutils.h"
#include <QDir>
NAMESPACE_BEGIN {namespace scanner{
    ImageThread::ImageThread(QObject* parent):QObject(parent){
        
    }
    
    ImageThread::~ImageThread(){
        stop();
        if(thread_.joinable()) thread_.join();
    }

    void ImageThread::stop(){
        isStopped_ = true;
        waitCondition_.wakeAll();
    }

    void ImageThread::start(){
        isStopped_ = false;
        if(!isStopped_){
            thread_ = std::thread(&ImageThread::work, this);
        }
    }

    void ImageThread::work(){
        while(!isStopped_){
            ReconstructionFramePtr frame; {
                QMutexLocker lock(&mutex_);
                if(frameQueue_.empty() && !isStopped_){
                    waitCondition_.wait(&mutex_);
                    continue;
                }

                if(isStopped_){
                    break;
                }

                frame = frameQueue_.front();
                frameQueue_.pop();
                std::cout << "ImageThread::work() -> Process frame " << frame->scanIndex << std::endl;
                //应对保存图像的情况
                if(frame->saveImage){ 
                    auto mat = frame->image.clone();
                    QString format = frame->imageFormat.trimmed().toLower();
                    if(format.startsWith('.')) format.remove(0, 1);
                    if(format.isEmpty()) format = "bmp";
                    QDir dir(frame->savePath);
                    if(!dir.exists()){
                        dir.mkpath(".");
                    }
                    auto savepath = frame->savePath + "/" + QString::number(frame->scanIndex) + "." + format;
                    try {
                        if(!detail::saveImageFile(savepath, mat)){
                            qWarning() << "扫描图片保存失败:" << savepath;
                        }
                    } catch (const cv::Exception& e) {
                        qWarning() << "扫描图片保存异常:" << savepath << e.what();
                    }
                }
            }
            // 处理 frame
        }
    }

    void ImageThread::enqueueFrame(ReconstructionFramePtr frame){
        QMutexLocker lock(&mutex_);
        frameQueue_.push(frame);
        waitCondition_.wakeOne();
    }
}}
