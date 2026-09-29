/** 冻结文件后复用共享镜像校验；主机不会借助这些存储回调写入任何设备。 */
#include "upgrade_plan.h"
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QSaveFile>
#include <cstring>
#include <stdexcept>
extern "C"
{
#include "shared/image/image_transfer.h"
#include "shared/protocol/yg_protocol_byte_order.h"
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

void write_new(const QString &path, const QByteArray &bytes)
{
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit())
    {
        throw std::runtime_error("任务导出失败；保留部分输出供检查，换用新的目录重试");
    }
}

QString sha256(const QByteArray &bytes)
{
    return QString::fromLatin1(QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex());
}
} // namespace

QJsonObject Upgrade_Target()
{
    QFile file(":/contracts/vector_mini_st_g431.json");
    if (!file.open(QIODevice::ReadOnly))
    {
        throw std::runtime_error("缺少共享目标资源");
    }
    const auto document = QJsonDocument::fromJson(file.readAll());
    if (!document.isObject() || document.object()["constants"].toObject().isEmpty())
    {
        throw std::runtime_error("共享目标资源格式损坏");
    }
    return document.object();
}

UpgradePlan Upgrade_Inspect(UpgradeMode mode,
                            const QString &loader_path,
                            const QString &app_path,
                            bool synthetic)
{
    UpgradePlan plan;
    plan.mode = mode;
    plan.synthetic = synthetic;
    try
    {
        if (mode != UpgradeMode::Loader && mode != UpgradeMode::App &&
            mode != UpgradeMode::Combined)
        {
            throw std::runtime_error("未知升级模式");
        }
        plan.target = Upgrade_Target();
        const auto &target = plan.target;
        const auto app_base = constant(target, "TARGET_APP_BASE");
        QJsonObject fixture;
        if (synthetic)
        {
            QFile file(":/fixtures/loader_image_v2.json");
            if (!file.open(QIODevice::ReadOnly))
            {
                throw std::runtime_error("缺少离线合成镜像");
            }
            fixture = QJsonDocument::fromJson(file.readAll()).object();
        }
        if (mode != UpgradeMode::App)
        {
            const auto base = constant(target, "TARGET_FLASH_BASE");
            const auto capacity = constant(target, "TARGET_LOADER_BYTES");
            QByteArray bytes;
            if (synthetic)
            {
                bytes.resize(64);
                bytes.fill(char(0xff));
                auto *output = reinterpret_cast<uint8_t *>(bytes.data());
                YgProtocol_WriteU32Le(output, constant(target, "TARGET_RAM_END"));
                YgProtocol_WriteU32Le(output + 4, (base + 16) | 1);
            }
            else
            {
                bytes = read_file(loader_path, capacity);
            }
            const auto *data = reinterpret_cast<const uint8_t *>(bytes.constData());
            const auto sp = YgProtocol_ReadU32Le(data);
            const auto entry = YgProtocol_ReadU32Le(data + 4);
            if (sp <= constant(target, "TARGET_RAM_BASE") ||
                sp > constant(target, "TARGET_RAM_END") || sp % 8 || !(entry & 1) ||
                (entry & ~1U) < base + 8 || (entry & ~1U) >= quint64(base) + bytes.size())
            {
                throw std::runtime_error("Loader 向量不属于当前 Flash/RAM 分区");
            }
            plan.regions.append({"Loader", synthetic ? "内置合成样本" : loader_path, base, bytes});
            plan.checks << "Loader 大小与向量检查通过；构建版本/布局兼容性仍需审核";
        }
        if (mode != UpgradeMode::Loader)
        {
            const auto bytes = synthetic
                                   ? QByteArray::fromHex(fixture["image_hex"].toString().toLatin1())
                                   : read_file(app_path, constant(target, "TARGET_APP_BYTES"));
            plan.manifest = synthetic
                                ? QByteArray::fromHex(fixture["manifest_hex"].toString().toLatin1())
                                : read_file(QFileInfo(app_path).dir().filePath("manifest.bin"), 96);
            if (plan.manifest.size() != 96)
            {
                throw std::runtime_error("APP 同目录必须有完整的 96B manifest.bin");
            }
            MemoryImage memory{bytes, app_base};
            auto config = image_config(target, &memory);
            LoaderImageIdentity identity{};
            if (!LoaderImage_DecodeManifest(
                    &config,
                    reinterpret_cast<const uint8_t *>(plan.manifest.constData()),
                    &identity) ||
                identity.bytes != quint32(bytes.size()) || !LoaderImage_Check(&config, &identity))
            {
                throw std::runtime_error("APP 包不兼容或 CRC/向量/描述符损坏；请使用当前打包工具");
            }
            plan.regions.append({"APP", synthetic ? "内置合成样本" : app_path, app_base, bytes});
            plan.checks << "APP 目标 / schema / ABI / 角色 / 长度 / 整包 CRC32 / 向量检查通过";
        }
        const auto parameter_base = constant(target, "TARGET_PARAM_BASE");
        for (const auto &region : plan.regions)
        {
            const quint64 limit = region.component == "Loader" ? app_base : parameter_base;
            if (quint64(region.address) + region.bytes.size() > limit)
            {
                throw std::runtime_error("任务写域跨越受保护分区");
            }
        }
        plan.transport = mode == UpgradeMode::App ? "CAN FD · 升级载荷 v2" : "SWD · 离线安装";
        plan.checks << "写域不包含参数区" << "本地校验完成；尚未核对实际设备与发布资格";
        if (mode == UpgradeMode::App)
        {
            plan.steps = {"识别配套 APP/Loader 与目标兼容性",
                          "确认停机，必要时 Type 56 进入 Loader",
                          "发现 HOST_NONCE，生成非零随机 nonce",
                          "BEGIN 后等待 RECEIVING",
                          "连续 WRITE / STATUS，核对已读回偏移",
                          "FINISH 后等待 VERIFIED",
                          "ACTIVATE 后重新发现并核对 APP 身份"};
        }
        else
        {
            plan.steps = {"核对目标与 Loader 构建信息，保持设备不运行",
                          "备份并核对完整参数区",
                          "只擦写任务内的独立区域，禁止全片擦除",
                          "按区域读回并核对镜像",
                          "核对参数备份与现有 schema 兼容性",
                          "释放复位并重新发现 APP/Loader"};
        }
        plan.valid = true;
    }
    catch (const std::exception &error)
    {
        plan.error = QString::fromUtf8(error.what());
        plan.regions.clear();
        plan.manifest.clear();
    }
    return plan;
}

QString Upgrade_Export(const UpgradePlan &plan, const QString &directory)
{
    if (!plan.valid || plan.regions.isEmpty() || !QFileInfo(directory).isAbsolute() ||
        QFileInfo::exists(directory))
    {
        throw std::runtime_error("请选择不存在的绝对输出目录，并先完成文件校验");
    }
    if (!QDir().mkpath(directory))
    {
        throw std::runtime_error("无法创建输出目录");
    }
    QJsonArray regions;
    for (const auto &region : plan.regions)
    {
        const auto filename = region.component == "Loader" ? "loader.bin" : "image.bin";
        write_new(QDir(directory).filePath(filename), region.bytes);
        regions.append(
            QJsonObject{{"component", region.component},
                        {"file", filename},
                        {"address", QString("0x%1").arg(region.address, 8, 16, QChar('0'))},
                        {"bytes", region.bytes.size()},
                        {"sha256", sha256(region.bytes)}});
    }
    if (!plan.manifest.isEmpty())
    {
        write_new(QDir(directory).filePath("manifest.bin"), plan.manifest);
    }
    const QJsonObject job{
        {"format", "yg-mc-host-maintenance-job-v1"},
        {"status", "prepared_only"},
        {"synthetic", plan.synthetic},
        {"hardware_accessed", false},
        {"release_approved", false},
        {"ready_to_execute", false},
        {"transport", plan.transport},
        {"target", plan.target},
        {"target_sha256", sha256(QJsonDocument(plan.target).toJson(QJsonDocument::Compact))},
        {"regions", regions},
        {"checks", QJsonArray::fromStringList(plan.checks)},
        {"steps", QJsonArray::fromStringList(plan.steps)},
        {"manifest_sha256", sha256(plan.manifest)},
        {"parameter_region_policy", "preserve_and_verify"},
        {"loader_build_compatibility", "requires_independent_review"}};
    const auto path = QDir(directory).filePath("job.json");
    write_new(path, QJsonDocument(job).toJson());
    return path;
}
} // namespace yg::host
