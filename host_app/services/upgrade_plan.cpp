/** 本地 APP 镜像校验复用共享 Loader 核心，不执行设备写入。 */
#include "upgrade_plan.h"
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <cstring>
#include <stdexcept>
extern "C"
{
#include "shared/image/image_transfer.h"
}

namespace yg::host
{
namespace
{
struct MemoryImage
{
    QByteArray bytes;
    quint32 base;
};

quint32 constant(const QJsonObject &target, const char *name)
{
    QString text = target["constants"].toObject()[name].toString();
    if (text.endsWith('U'))
    {
        text.chop(1);
    }
    bool valid = false;
    const auto value = text.toUInt(&valid, 0);
    if (!valid || value == 0)
    {
        throw std::runtime_error(QString("目标常量无效：%1").arg(name).toStdString());
    }
    return value;
}

QByteArray read_file(const QString &path, quint32 maximum)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly) || file.size() < 8 || file.size() > maximum)
    {
        throw std::runtime_error("文件不存在、过短或超过当前分区容量");
    }
    const auto bytes = file.readAll();
    if (bytes.size() != file.size())
    {
        throw std::runtime_error("文件读取不完整，请重新选择文件");
    }
    return bytes;
}

bool memory_read(void *context, uint32_t address, void *output, uint32_t count)
{
    const auto &image = *static_cast<MemoryImage *>(context);
    if (address < image.base || count > quint32(image.bytes.size()) ||
        address - image.base > quint32(image.bytes.size()) - count)
    {
        return false;
    }
    std::memcpy(output, image.bytes.constData() + address - image.base, count);
    return true;
}

bool deny_erase(void *, uint32_t, uint32_t)
{
    return false;
}
bool deny_program(void *, uint32_t, const void *, uint32_t)
{
    return false;
}
bool deny_mutation(void *)
{
    return false;
}
void no_watchdog(void *)
{
}

LoaderImageConfig image_config(const QJsonObject &target, MemoryImage *image)
{
    LoaderImageConfig config{};
    config.io = {image,
                 constant(target, "TARGET_FLASH_BASE"),
                 constant(target, "TARGET_FLASH_BYTES"),
                 constant(target, "TARGET_ERASE_BYTES"),
                 constant(target, "TARGET_PROGRAM_BYTES"),
                 memory_read,
                 deny_erase,
                 deny_program};
    config.app = {constant(target, "TARGET_APP_BASE"), constant(target, "TARGET_APP_BYTES")};
    config.layout = constant(target, "TARGET_LAYOUT");
    config.mcu = constant(target, "TARGET_MCU");
    config.product = constant(target, "TARGET_PRODUCT");
    config.hardware = constant(target, "TARGET_HARDWARE");
    config.param_abi = constant(target, "TARGET_PARAM_ABI");
    config.ram_start = constant(target, "TARGET_RAM_BASE");
    config.ram_end = constant(target, "TARGET_RAM_END");
    config.descriptor_offset = constant(target, "TARGET_DESCRIPTOR_OFFSET");
    config.schema = quint16(constant(target, "TARGET_SCHEMA"));
    config.allowed = deny_mutation;
    config.watchdog = no_watchdog;
    return config;
}
} // namespace

UpgradePlan Upgrade_Inspect(const QString &app_path)
{
    UpgradePlan plan;
    try
    {
        if (QFileInfo(app_path).fileName() != "image.bin")
        {
            throw std::runtime_error("请选择打包后的 image.bin");
        }
        QFile resource(":/contracts/vector_mini_st_g431.json");
        if (!resource.open(QIODevice::ReadOnly))
        {
            throw std::runtime_error("缺少共享目标资源");
        }
        const auto document = QJsonDocument::fromJson(resource.readAll());
        if (!document.isObject())
        {
            throw std::runtime_error("共享目标资源格式损坏");
        }
        const auto target = document.object();
        const auto app_base = constant(target, "TARGET_APP_BASE");
        const auto image = read_file(app_path, constant(target, "TARGET_APP_BYTES"));
        const auto manifest = read_file(QFileInfo(app_path).dir().filePath("manifest.bin"), 96);
        if (manifest.size() != 96)
        {
            throw std::runtime_error("APP 同目录必须有完整的 96B manifest.bin");
        }
        MemoryImage memory{image, app_base};
        auto config = image_config(target, &memory);
        LoaderImageIdentity identity{};
        if (!LoaderImage_DecodeManifest(
                &config, reinterpret_cast<const uint8_t *>(manifest.constData()), &identity) ||
            identity.bytes != quint32(image.size()) || !LoaderImage_Check(&config, &identity))
        {
            throw std::runtime_error("APP 包不兼容或 CRC/向量/描述符损坏");
        }
        if (quint64(app_base) + image.size() > constant(target, "TARGET_PARAM_BASE"))
        {
            throw std::runtime_error("APP 镜像跨越参数分区");
        }
        plan.checks << QString("APP v%1.%2.%3 · MCU 0x%4 · layout %5 · schema %6")
                           .arg(identity.version >> 16U)
                           .arg((identity.version >> 8U) & 0xFFU)
                           .arg(identity.version & 0xFFU)
                           .arg(constant(target, "TARGET_MCU"), 8, 16, QChar('0'))
                           .arg(constant(target, "TARGET_LAYOUT"))
                           .arg(constant(target, "TARGET_SCHEMA"));
        plan.checks << QString("APP 0x%1 · %2 B · CRC32 0x%3 · ABI 0x%4")
                           .arg(app_base, 8, 16, QChar('0'))
                           .arg(identity.bytes)
                           .arg(identity.crc32, 8, 16, QChar('0'))
                           .arg(constant(target, "TARGET_PARAM_ABI"), 8, 16, QChar('0'));
        plan.valid = true;
    }
    catch (const std::exception &error)
    {
        plan.error = QString::fromUtf8(error.what());
    }
    return plan;
}
} // namespace yg::host
