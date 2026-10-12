#ifndef IMAGE_THREAD_H
#define IMAGE_THREAD_H

#include <QThread>
#include <QDebug>
#include <QDateTime>
#include <QMutex>
#include <thread>
#include <queue>
#include <condition_variable>
#include <QWaitCondition>
#include "utils.h"
#include "scanner/scanner.h"

NAMESPACE_BEGIN {namespace scanner{
    /**
     * @brief 后台处理与图像相关的耗时任务
     */
    class ImageThread : public QObject {
        Q_OBJECT
    public:
        ImageThread(QObject* parent = nullptr);
        ~ImageThread();
    public slots:     
        void stop();
        void enqueueFrame(ReconstructionFramePtr frame);
        void start();

    private:
        void work();
        std::thread thread_;
        bool isStopped_;
        QMutex mutex_;
        QWaitCondition waitCondition_;
        std::queue<ReconstructionFramePtr> frameQueue_;

    };
}}

#endif // IMAGE_THREAD_H