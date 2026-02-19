import torch

def test_cuda():
    if torch.cuda.is_available():
        print(f"GPU: {torch.cuda.get_device_name(torch.cuda.current_device())}")
        return True
    else:
        return False

if __name__ == "__main__":
    success = test_cuda()
    exit(0 if success else 1)
