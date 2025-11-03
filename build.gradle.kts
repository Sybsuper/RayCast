plugins {
    id("java-library")
}

java {
    toolchain {
        languageVersion.set(JavaLanguageVersion.of(25))
    }
}

version = "dev"

repositories {
    mavenCentral()
    maven("https://jitpack.io/")
}

dependencies {
    compileOnly("net.minestom:minestom:2025.10.18-1.21.10")
    testImplementation("net.minestom:minestom:2025.10.18-1.21.10")
    implementation("org.apache.commons:commons-geometry-euclidean:1.0")
    compileOnly("org.jetbrains:annotations:24.1.0")
}

tasks.withType<Test> {
    enabled = false
}
