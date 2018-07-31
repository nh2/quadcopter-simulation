{-# LANGUAGE OverloadedStrings #-}
{-# LANGUAGE NamedFieldPuns #-}
{-# LANGUAGE ScopedTypeVariables #-}

module Main where

import Control.Concurrent.Async
-- import Data.Char (ord)
-- import qualified Data.ByteString as BS
-- import qualified Data.ByteString.Char8 as BS8
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

-- import System.Clock
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
                           , copterPos :: V3 Double
                           , deviceAccel :: V3 Int
                           , accel :: V3 Double
                           , speed :: V3 Double
                           }

toVertex :: (Real a, Fractional b) => V3 a -> Vertex3 b
toVertex xyz = (\(V3 x y z) -> Vertex3 x y z) $ fmap realToFrac xyz

setCamera :: PlayerState -> IO ()
setCamera (Running (V3 x y z) _ euler) =
  GLUT.lookAt (toVertex xyz0) (toVertex target) (Vector3 0 0 (-1))
  where
    xyz0 = V3 x y (z-faceHeight)
    target = xyz0 + rotVecByEulerB2A euler (V3 1 0 0)

initialCopterPos :: V3 Double
initialCopterPos = V3 0 0 (-1) -- start above the ground

flipX :: (Num a) => V3 a -> V3 a
flipX (V3 x y z) = V3 (-x) y z

boundAbs :: (Ord a, Num a) => a -> a -> a
boundAbs cap val
  | val < -cap = -cap
  | val >  cap =  cap
  | otherwise  =  val

boundLower :: (Ord a, Num a) => a -> a -> a
boundLower cap val
  | val < -cap = -cap
  | otherwise  =  val

boundUpper :: (Ord a, Num a) => a -> a -> a
boundUpper cap val
  | val > cap = cap
  | otherwise = val

simfun :: Float -> GameState -> IO GameState
simfun _ (GameState (Running pos _ euler0@(Euler yaw _ _)) keys lmp copterPos deviceAccel accel speed) = do
  (x', y') <- do
    Size x y <- GLUT.get GLUT.windowSize
    let x' = (fromIntegral x) `div` 2
        y' = (fromIntegral y) `div` 2
    return (x', y')

  when (Just (x',y') /= lmp) (GLUT.pointerPosition $= (Position x' y'))

  let newAccel = (if thrust then -0.5 else 0) *^ accel + V3 0 0 (0.0002)
  let newSpeed =
        boundAbs 0.05 $
          speed + flipX newAccel
  let newCopterPos =
        copterPos + newSpeed

  let touchesGround = let V3 _ _ z = newCopterPos in z > 0

  let collidedSpeed = if touchesGround then 0 else newSpeed
  let collidedCopterPos
        | touchesGround = let V3 x y _ = newCopterPos in V3 x y 0
        | otherwise     = newCopterPos

  let onReset resetVal val = if reset then resetVal else val

  let finalSpeed = onReset (V3 0 0 0) collidedSpeed
  let finalCopterPos = onReset initialCopterPos collidedCopterPos

  return $ GameState (Running (pos + (ts *^ v)) v euler0) keys (Just (x',y')) finalCopterPos deviceAccel accel finalSpeed
  where
    v = rotateXyzAboutZ (V3 (w-s) (d-a) 0) yaw
      where
        w = if Set.member (Char 'w') keys then 3 else 0
        a = if Set.member (Char 'a') keys then 3 else 0
        s = if Set.member (Char 's') keys then 3 else 0
        d = if Set.member (Char 'd') keys then 3 else 0

    thrust = Set.member (Char ' ') keys
    reset = Set.member (Char 'r') keys

keyMouseCallback :: GameState -> Key -> KeyState -> Modifiers -> Position -> GameState
keyMouseCallback state0 key keystate _ _
  | keystate == Down = state0 {keySet = Set.insert key (keySet state0)}
  | keystate == Up   = state0 {keySet = Set.delete key (keySet state0)}
  | otherwise        = state0

motionCallback :: Bool -> GameState -> Position -> GameState
motionCallback _ state0@(GameState (Running pos v (Euler yaw0 pitch0 _)) _ lmp _ _ _ _) (Position x y) =
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


drawfun :: GameState -> VisObject Double
drawfun GameState{ playerState = Running _ _ _, copterPos, accel = V3 x y _z } =
  VisObjects $ [axes, box, plane]
  where
    axes = Axes (0.5, 15)
    box =
      Trans copterPos $
        RotEulerDeg (Euler{ eYaw = 0
                          , ePitch = (-x) * 0.9 / 0.00001
                          , eRoll = (-y) * 0.9 / 0.00001
                          }) $
          Box (0.2, 0.2, 0.2) Solid (makeColor 0 1 1 1)
    plane = Plane (V3 0 0 1) (makeColor 1 1 1 1) (makeColor 0.4 0.6 0.65 0.4)


main :: IO ()
main = do
  -- let port = "COM3"           -- Windows
  let port = "/dev/ttyUSB0"   -- Linux
  -- s <- openSerial port defaultSerialSettings
  h <- hOpenSerial port defaultSerialSettings
    { timeout = 1000
    , commSpeed = CS115200
    }

  deviceAccelRef <- newIORef $ V3 0 0 0

  let serialReaderThread = forever $ do
        -- -- recv s 1 >>= (\([c]) -> print (ord c)) . BS8.unpack
        -- x <- fromIntegral . (\([c]) -> ord c) . BS8.unpack <$> recv s 1
        -- print x

        -- case [x-100, 0, 0] of
        --   [x, y, z] -> do
        --     writeIORef deviceAccelRef (Accel x y z)
        --     nanos <- toNanoSecs <$> getTime Monotonic
        --     -- old <- atomicModifyIORef' deviceAccelRef $ \(Accel oldX oldY oldZ) ->
        --     --   let w = 0.2
        --     --       i old new = floor (fromIntegral old * (1-w) + fromIntegral new * w :: Double)
        --     --       a = Accel (i oldX x) (i oldY y) (i oldZ z)
        --     --   in (a, a)
        --     -- print (nanos `quot` 1000000)
        --     -- print (nanos `quot` 1000)
        --     -- print old
        --     print (x, y, z)
        --   _ -> return ()

        -- print "here"
        l <- hGetLine h
        -- print l
        case catMaybes $ map (readMaybe . T.unpack) $ T.splitOn " " $ T.strip (T.pack l) of
          -- [x, y, z :: Int] -> do
          x:y:(z :: Int):_ -> do
            writeIORef deviceAccelRef (V3 x y z)
            -- nanos <- toNanoSecs <$> getTime Monotonic
            -- old <- atomicModifyIORef' deviceAccelRef $ \(V3 oldX oldY oldZ) ->
            --   let w = 0.2
            --   -- let w = 1
            --       i old new = floor (fromIntegral old * (1-w) + fromIntegral new * w :: Double)
            --       a = Accel (i oldX x) (i oldY y) (i oldZ z)
            --   in (a, a)
            -- print old
            -- print (nanos `quot` 1000000)
            -- print (x, y, z)
          _ -> return ()


  race_ serialReaderThread $ do

    let state0 =
          GameState
            { playerState = Running (V3 (-2) 0 0) 0 (Euler 0 0 0)
            , keySet = Set.empty
            , lastMousePos = Nothing
            , copterPos = initialCopterPos
            , deviceAccel = V3 0 0 0
            , accel = V3 0 0 0
            , speed = V3 0 0 0
            }

        setCam GameState{ playerState } = setCamera playerState

        simfun' time gameState@GameState{ accel } = do
          deviceAccel <- readIORef deviceAccelRef

          let interpolate w a b = (1-w) *^ a + w *^ b
          let w = 0.2
          let newAccel = interpolate w accel (0.00001 *^ (fromIntegral <$> deviceAccel))

          simfun time gameState{ deviceAccel, accel = newAccel }

        drawfun' :: GameState -> IO (VisObject Double, Maybe Cursor)
        drawfun' x = return (drawfun x, Just None)

    _ <- initThreads
    playIO (defaultOpts {optWindowName = "play test"}) ts state0 drawfun' simfun' setCam
      (Just keyMouseCallback) (Just (motionCallback True)) (Just (motionCallback False))
