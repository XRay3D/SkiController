import qbs 1.0

Product {
    type: [
        "application",
        "flash",
    ]

    Depends { name:"cpp" }
    property string iarInc: "C:/Program Files/IAR Systems/Embedded Workbench 4.0 Evaluation/ARM/inc"

    cpp.defines: [
        "__STDC_CONSTANT_MACROS",
        "__INT8_T_TYPE__",
        "__INT16_T_TYPE__",
        "__INT32_T_TYPE__",
        "__INT64_T_TYPE__"
    ]

    cpp.debugInformation: true
    cpp.executableSuffix: ".elf"
    cpp.positionIndependentCode: false

    cpp.commonCompilerFlags: [
        //        "-fdata-sections",
        //        "-ffunction-sections",
        //        "-flto",
        //        "-fno-asynchronous-unwind-tables",
        //        //"-fno-inline",
        //        "-mcpu=cortex-m3",
        //        "-mfpu=vfp",
        //        "-msoft-float",
        //        "-mthumb",
        //        "-Os",
        //        "-std=c++11",
        //        "-Wl,--gc-sections",
        //        "-Wl,--start-group",
        //        "-Wl,--strip-all",
    ]
    cpp.linkerFlags:[
        //        "--gc-sections",
        //        "--start-group",
        //        "-flto",
        //        "-lc",
        //        "-lgcc",
        //        "-lm",
        //        "-lnosys",
        //        "-lstdc++",
        //        "-shared",
        //        "-T" + path + "/STM32F407VG_FLASH.ld",
    ]
    cpp.includePaths: [
        iarInc,
        "app/",
        "cc1101_lib/",
        "modules/",
        "sensors/",
    ]
    files: [
        "*/*h",
        "*/*c",
        "*/*cpp",
        "*/*s79",
    ]

    //    Group {
    //        //add  .rel.plt : { *(.rel.plt) } before .text : if needed
    //        name: "ldscripts"
    //        prefix: ""
    //        files: "*.ld"
    //    }

    //    Properties
    //    {
    //        condition: qbs.buildVariant === "debug"
    //        cpp.defines: outer.concat(["DEBUG=1"])
    //        cpp.debugInformation: true
    //        cpp.optimization: "none"
    //    }

    //    Properties
    //    {
    //        condition: qbs.buildVariant === "release"
    //        cpp.debugInformation: false
    //        cpp.optimization: "small"
    //    }

    //    Group {
    //        qbs.install: true
    //        fileTagsFilter: "application"
    //    }

    //    Rule {
    //        id: size
    //        inputs: "application"
    //        Artifact {
    //            fileTags: ["size"]
    //            filePath: "-"
    //        }
    //        prepare: {
    //            var args = [input.filePath];
    //            var cmd = new Command("arm-none-eabi-size", args);
    //            cmd.description = "File size: " + args;
    //            cmd.highlight = "linker";
    //            return cmd;
    //        }
    //    }

    //    Rule
    //    {
    //        inputs: ["application"]

    //        Artifact
    //        {
    //            filePath: project.path + "/bin/" + input.baseName + ".hex"
    //            fileTags: "flash"
    //        }

    //        prepare:
    //        {
    //            var sizePath = "arm-none-eabi-size.exe";
    //            var objcopyPath = "arm-none-eabi-objcopy.exe";

    //            var argsSize = [input.filePath];
    //            var argsObjcopy = ["-O", "ihex", input.filePath, output.filePath];

    //            var cmdSize = new Command(sizePath, argsSize);
    //            var cmdObjcopy = new Command(objcopyPath, argsObjcopy);

    //            cmdSize.description = "Size of sections:";
    //            cmdSize.highlight = "linker";

    //            cmdObjcopy.description = "convert to bin...";
    //            cmdObjcopy.highlight = "linker";

    //            return [cmdSize, cmdObjcopy];
    //        }
    //    }
}


///////////////////////////
//Rule {
//    id: hex
//    inputs: "application"
//    Artifact {
//        fileTags: ["hex"]
//        filePath: FileInfo.baseName(input.filePath) + ".hex"
//    }
//    prepare: {
//        var args = ["-O", "ihex", input.filePath, output.filePath];
//        var cmd = new Command("arm-none-eabi-objcopy", args);
//        cmd.description = "converting to hex: "+FileInfo.fileName(input.filePath);
//        cmd.highlight = "linker";
//        return cmd;

//    }
//}

//Rule {
//    id: bin
//    inputs: "application"
//    Artifact {
//        fileTags: ["bin"]
//        filePath: FileInfo.baseName(input.filePath) + ".bin"
//    }
//    prepare: {
//        var args = ["-O", "binary", input.filePath, output.filePath];
//        var cmd = new Command("arm-none-eabi-objcopy", args);
//        cmd.description = "converting to bin: "+FileInfo.fileName(input.filePath);
//        cmd.highlight = "linker";
//        return cmd;

//    }
//}

//Rule {
//    id: size
//    inputs: "application"
//    Artifact {
//        fileTags: ["size"]
//        filePath: "-"
//    }
//    prepare: {
//        var args = [input.filePath];
//        var cmd = new Command("arm-none-eabi-size", args);
//        cmd.description = "File size: " + FileInfo.fileName(input.filePath);
//        cmd.highlight = "linker";
//        return cmd;
//    }
//}

///////////////////////////////////////////////////////////////////////////////
//import qbs

//Product
//{
//    type: ["application", "flash"]
//    Depends { name: "cpp" }

//    cpp.defines: ["STM32F10X_LD_VL"]
//    cpp.positionIndependentCode: false
//    cpp.enableExceptions: false
//    cpp.executableSuffix: ".elf"
//    cpp.driverFlags:
//        [
//        "-mthumb",
//        "-mcpu=cortex-m3",
//        "-mfloat-abi=soft",
//        "-fno-strict-aliasing",
//        "-g3",
//        "-Wall",
//        "-mfpu=vfp",
//        "-O0",
//        "-flto",
//    ]

//    cpp.commonCompilerFlags:
//        [
//        "-fdata-sections",
//        "-ffunction-sections",
//        "-fno-inline",
//        "-std=c++11",
//        "-flto"
//    ]

//    cpp.linkerFlags:
//        [
//        "--specs=nano.specs",
//        "-Wl,--start-group",
//        "-Wl,--gc-sections",
//        "-T" + path + "/src/system/linker/stm32f10x_flash.ld",
//        "-lnosys",
//        "-lgcc",
//        "-lc",
//        "-lstdc++",
//        "-lm"
//    ]

//    cpp.includePaths:
//        [
//        "src/system/cmsis",
//        "src/system/cmsis_boot",
//        "src/system/cmsis_boot/statup"
//    ]

//    files:
//        [
//        "*.h",
//        "*.h",
//        "*.h",
//        "src/system/cmsis_boot/*.c",
//        "src/system/cmsis_boot/startup/*.c",
//        "src/main.cpp"
//    ]

//    Properties
//    {
//        condition: qbs.buildVariant === "debug"
//        cpp.defines: outer.concat(["DEBUG=1"])
//        cpp.debugInformation: true
//        cpp.optimization: "none"
//    }

//    Properties
//    {
//        condition: qbs.buildVariant === "release"
//        cpp.debugInformation: false
//        cpp.optimization: "small"
//    }

//    Rule
//    {
//        inputs: ["application"]

//        Artifact
//        {
//            filePath: project.path + "/debug/bin/" + input.baseName + ".hex"
//            fileTags: "flash"
//        }

//        prepare:
//        {
//            var sizePath = "c:/development/gcc-arm/bin/arm-none-eabi-size.exe";
//            var objcopyPath = "c:/development/gcc-arm/bin/arm-none-eabi-objcopy.exe";
//            var configStlinkPath = "c:/development/openocd_0_10_0/scripts/interface/stlink-v2.cfg";
//            var configStm32Path = "c:/development/openocd_0_10_0/scripts/target/stm32f1x.cfg";
//            var flashPath = "c:/development/openocd_0_10_0/bin/openocd.exe";

//            var argsSize = [input.filePath];
//            var argsObjcopy = ["-O", "ihex", input.filePath, output.filePath];

//            var argsFlashing =
//                    [
//                        "-f", configStlinkPath,
//                        "-f", configStm32Path,
//                        "-c", "init",
//                        "-c", "halt",
//                        "-c", "flash write_image erase " + input.filePath,
//                        "-c", "reset",
//                        "-c", "shutdown"
//                    ];

//            var cmdSize = new Command(sizePath, argsSize);
//            var cmdObjcopy = new Command(objcopyPath, argsObjcopy);
//            var cmdFlash = new Command(flashPath, argsFlashing);

//            cmdSize.description = "Size of sections:";
//            cmdSize.highlight = "linker";

//            cmdObjcopy.description = "convert to bin...";
//            cmdObjcopy.highlight = "linker";

//            cmdFlash.description = "download firmware to uC...";
//            cmdFlash.highlight = "linker";

//            return [cmdSize, cmdObjcopy, cmdFlash];
//        }
//    }
//}
