package com.glassimaging.climage

import android.content.res.AssetManager
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import com.glassimaging.climage.databinding.ActivityMainBinding

class TestCLImage : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        // Load test image from assets
        val baboonBitmap = this.assets.open(baboonPath).use { BitmapFactory.decodeStream(it) }

        // Allocate output result
        val outputBitmap =
            Bitmap.createBitmap(baboonBitmap.width, baboonBitmap.height, Bitmap.Config.ARGB_8888)

        // Process input image to generate output
        testCLImage(this.assets, baboonBitmap, outputBitmap)

        // Display output image on screen
        binding.imageView.setImageBitmap(outputBitmap)
    }

    /**
     * A native method that is implemented by the 'climage' native library,
     * which is packaged with this application.
     */
    external fun testCLImage(assetManager: AssetManager, inputImage : Bitmap, outputImage : Bitmap): Int

    companion object {
        // Used to load the 'climage' library on application startup.
        init {
            System.loadLibrary("climage")
        }

        private const val baboonPath = "baboon.png"
    }
}