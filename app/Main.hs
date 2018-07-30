{-# LANGUAGE OverloadedStrings #-}

module Main where

import Control.Concurrent.Async
import Data.IORef
import qualified Data.Text as T
import Data.Maybe (catMaybes)
import Text.Read (readMaybe)
import Linear ( V3(..), (*^) )
import qualified Data.Set as Set
import Graphics.X11 ( initThreads )
import Graphics.UI.GLUT ( Cursor(..), Key(..), KeyState(..), Modifiers(..), Position(..)
                        , Size(..), Vector3(..), Vertex3(..)
                        , GLint
                        , ($=)
                        )
import qualified Graphics.UI.GLUT as GLUT

import SpatialMath ( Euler(..), rotateXyzAboutZ, rotVecByEulerB2A )
import Vis

import System.Clock
import System.IO
import System.Hardware.Serialport

import Control.Monad ( when, forever )

ts :: Double
ts = 0.01

faceHeight :: Double
faceHeight = 1.5

data PlayerState = Running (V3 Double) (V3 Double) (Euler Double)

data GameState = GameState { playerState :: PlayerState
                           , keySet :: Set.Set Key
                           , lastMousePos :: Maybe (GLint,GLint)
                           }

toVertex :: (Real a, Fractional b) => V3 a -> Vertex3 b
toVertex xyz = (\(V3 x y z) -> Vertex3 x y z) $ fmap realToFrac xyz

setCamera :: PlayerState -> IO ()
setCamera (Running (V3 x y z) _ euler) =
  GLUT.lookAt (toVertex xyz0) (toVertex target) (Vector3 0 0 (-1))
  where
    xyz0 = V3 x y (z-faceHeight)
    target = xyz0 + rotVecByEulerB2A euler (V3 1 0 0)

simfun :: Float -> GameState -> IO GameState
simfun _ (GameState (Running pos _ euler0@(Euler yaw _ _)) keys lmp) = do
  Size x y <- GLUT.get GLUT.windowSize
  let x' = (fromIntegral x) `div` 2
      y' = (fromIntegral y) `div` 2

  when (Just (x',y') /= lmp) (GLUT.pointerPosition $= (Position x' y'))
  return $ GameState (Running (pos + (ts *^ v)) v euler0) keys (Just (x',y'))
  where
    v = rotateXyzAboutZ (V3 (w-s) (d-a) 0) yaw
      where
        w = if Set.member (Char 'w') keys then 3 else 0
        a = if Set.member (Char 'a') keys then 3 else 0
        s = if Set.member (Char 's') keys then 3 else 0
        d = if Set.member (Char 'd') keys then 3 else 0

keyMouseCallback :: GameState -> Key -> KeyState -> Modifiers -> Position -> GameState
keyMouseCallback state0 key keystate _ _
  | keystate == Down = state0 {keySet = Set.insert key (keySet state0)}
  | keystate == Up   = state0 {keySet = Set.delete key (keySet state0)}
  | otherwise        = state0

motionCallback :: Bool -> GameState -> Position -> GameState
motionCallback _ state0@(GameState (Running pos v (Euler yaw0 pitch0 _)) _ lmp) (Position x y) =
  state0 {playerState = newPlayerState, lastMousePos = Just (x,y)}
  where
    (x0,y0) = case lmp of Nothing -> (x,y)
                          Just (x0',y0') -> (x0',y0')
    newPlayerState = Running pos v (Euler yaw pitch 0)
    dx = 0.002*realToFrac (x - x0)
    dy = 0.002*realToFrac (y - y0)
    yaw = yaw0 + dx
    pitch = bound (-89) 89 (pitch0 - dy)
    bound min' max' val
      | val < min' = min'
      | val > max' = max'
      | otherwise  = val


data Accel =
  Accel
    { accelX :: Int
    , accelY :: Int
    , accelZ :: Int
    } deriving (Eq, Ord, Show)


drawfun :: GameState -> Accel -> VisObject Double
drawfun (GameState (Running _ _ _) _ _) (Accel x y z) =
  VisObjects $ [axes,box, plane]
  where
    x' = -1
    axes = Axes (0.5, 15)
    box =
      Trans (V3 0 0 x') $
        RotEulerDeg (Euler{ eYaw = 0
                          , ePitch = fromIntegral (-x) * 0.9
                          , eRoll = fromIntegral (-y) * 0.9
                          }) $
          Box (0.2, 0.2, 0.2) Solid (makeColor 0 1 1 1)
    plane = Plane (V3 0 0 1) (makeColor 1 1 1 1) (makeColor 0.4 0.6 0.65 0.4)


main :: IO ()
main = do
  -- let port = "COM3"           -- Windows
  let port = "/dev/ttyUSB0"   -- Linux
  h <- hOpenSerial port defaultSerialSettings
    { timeout = 1000
    , commSpeed = CS115200
    }

  accelRef <- newIORef $ Accel 0 0 0

  let serialReaderThread = forever $ do
        h <- hGetLine h
        case catMaybes $ map (readMaybe . T.unpack) $ T.splitOn " " $ T.strip (T.pack h) of
          [x, y, z] -> do
            -- writeIORef accelRef (Accel x y z)
            nanos <- toNanoSecs <$> getTime Monotonic
            old <- atomicModifyIORef' accelRef $ \(Accel oldX oldY oldZ) ->
              let w = 0.2
                  i old new = floor (fromIntegral old * (1-w) + fromIntegral new * w :: Double)
                  a = Accel (i oldX x) (i oldY y) (i oldZ z)
              in (a, a)
            print (nanos `quot` 1000000)
            print old
            print (x, y, z)
          _ -> return ()


  race_ serialReaderThread $ do

    let state0 = GameState (Running (V3 (-2) 0 0) 0 (Euler 0 0 0)) (Set.empty) Nothing
        setCam (GameState x _ _) = setCamera x
        drawfun' x = do
          accel <- readIORef accelRef
          return (drawfun x accel, Just None)
    _ <- initThreads
    playIO (defaultOpts {optWindowName = "play test"}) ts state0 drawfun' simfun setCam
      (Just keyMouseCallback) (Just (motionCallback True)) (Just (motionCallback False))
