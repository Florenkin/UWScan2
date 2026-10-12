#include "setting/settingsmanager.h"
#include <QCoreApplication>
NAMESPACE_BEGIN{ namespace setting{
    SettingsManager::SettingsManager(QObject *parent): QObject(parent), settings(QSettings::IniFormat, QSettings::UserScope,
                QCoreApplication::organizationName(),
                QCoreApplication::applicationName()) {
    }

    SettingsManager * SettingsManager::getInstance(){
        static SettingsManager instance;
        return &instance;
    }

    QVariant SettingsManager::get(const QString& key, const QVariant& defaultValue) const{
        QMutexLocker locker(&mutex);
        return settings.value(key, defaultValue);
    }

    bool SettingsManager::contains(const QString& key) const{
        QMutexLocker locker(&mutex);
        return settings.contains(key);
    }
    
    void SettingsManager::set(const QString& key, const QVariant& value){
        QMutexLocker locker(&mutex);
        if(settings.value(key) == value){
            return;
        }
        settings.setValue(key, value);
        emit settingChanged(key, value);
    }
}}
