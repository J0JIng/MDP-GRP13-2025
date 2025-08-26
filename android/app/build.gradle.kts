plugins {
    alias(libs.plugins.android.application)
}

android {
    namespace = "com.example.androidmdpgrp13"
    compileSdk = 36

    defaultConfig {
        applicationId = "com.example.androidmdpgrp13"
        minSdk = 26
        targetSdk = 36
        versionCode = 1
        versionName = "1.0"
        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }

    // ✅ Java 17 for source & target
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    // (Optional) easier view access in Java
    buildFeatures {
        viewBinding = true
    }
}

dependencies {
    implementation(libs.appcompat)
    implementation(libs.material)
    implementation(libs.activity)          // fine for Java
    implementation(libs.constraintlayout)

    //for Navigation Component
    implementation("androidx.navigation:navigation-fragment:2.9.3")
    implementation("androidx.navigation:navigation-ui:2.9.3")
    implementation("androidx.fragment:fragment:1.7.1")

    testImplementation(libs.junit)
    androidTestImplementation(libs.ext.junit)
    androidTestImplementation(libs.espresso.core)
}
